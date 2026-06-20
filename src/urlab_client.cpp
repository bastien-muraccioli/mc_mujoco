#include "urlab_client.h"

#include <mc_rtc/logging.h>

#include <chrono>
#include <cstring>

namespace mc_mujoco
{

namespace
{

/** Pack a std::map<std::string, double> as a msgpack map */
template<typename Packer>
void pack_double_map(Packer & pk, const std::map<std::string, double> & m)
{
  pk.pack_map(static_cast<uint32_t>(m.size()));
  for(const auto & [k, v] : m)
  {
    pk.pack(k);
    pk.pack(v);
  }
}

/** Find a key in a msgpack map object, returns nullptr if absent or obj is not a map */
const msgpack::object * find(const msgpack::object & obj, const std::string & key)
{
  if(obj.type != msgpack::type::MAP)
  {
    return nullptr;
  }
  for(uint32_t i = 0; i < obj.via.map.size; ++i)
  {
    const auto & kv = obj.via.map.ptr[i];
    if(kv.key.type == msgpack::type::STR && std::string(kv.key.via.str.ptr, kv.key.via.str.size) == key)
    {
      return &kv.val;
    }
  }
  return nullptr;
}

std::string as_string(const msgpack::object & obj, const std::string & fallback = "")
{
  if(obj.type != msgpack::type::STR)
  {
    return fallback;
  }
  return {obj.via.str.ptr, obj.via.str.size};
}

bool as_bool(const msgpack::object & obj, bool fallback = false)
{
  if(obj.type != msgpack::type::BOOLEAN)
  {
    return fallback;
  }
  return obj.via.boolean;
}

int as_int(const msgpack::object & obj, int fallback = 0)
{
  switch(obj.type)
  {
    case msgpack::type::POSITIVE_INTEGER:
      return static_cast<int>(obj.via.u64);
    case msgpack::type::NEGATIVE_INTEGER:
      return static_cast<int>(obj.via.i64);
    default:
      return fallback;
  }
}

double as_double(const msgpack::object & obj, double fallback = 0.0)
{
  switch(obj.type)
  {
    case msgpack::type::FLOAT32:
    case msgpack::type::FLOAT64:
      return obj.via.f64;
    case msgpack::type::POSITIVE_INTEGER:
      return static_cast<double>(obj.via.u64);
    case msgpack::type::NEGATIVE_INTEGER:
      return static_cast<double>(obj.via.i64);
    default:
      return fallback;
  }
}

std::vector<double> as_double_vector(const msgpack::object & obj)
{
  std::vector<double> out;
  if(obj.type != msgpack::type::ARRAY)
  {
    return out;
  }
  out.reserve(obj.via.array.size);
  for(uint32_t i = 0; i < obj.via.array.size; ++i)
  {
    out.push_back(as_double(obj.via.array.ptr[i]));
  }
  return out;
}

/** Parse a {name: [doubles...]} map into std::map<std::string, std::vector<double>> */
std::map<std::string, std::vector<double>> as_named_vector_map(const msgpack::object * obj)
{
  std::map<std::string, std::vector<double>> out;
  if(!obj || obj->type != msgpack::type::MAP)
  {
    return out;
  }
  for(uint32_t i = 0; i < obj->via.map.size; ++i)
  {
    const auto & kv = obj->via.map.ptr[i];
    out[as_string(kv.key)] = as_double_vector(kv.val);
  }
  return out;
}

/** Parse a {origName: liveName} map into std::map<std::string,std::string> */
std::map<std::string, std::string> as_string_map(const msgpack::object * obj)
{
  std::map<std::string, std::string> out;
  if(!obj || obj->type != msgpack::type::MAP)
  {
    return out;
  }
  for(uint32_t i = 0; i < obj->via.map.size; ++i)
  {
    const auto & kv = obj->via.map.ptr[i];
    out[as_string(kv.key)] = as_string(kv.val);
  }
  return out;
}

} // namespace

URLabClient::URLabClient(const std::string & endpoint, int timeout_ms) : endpoint_(endpoint), timeout_ms_(timeout_ms)
{
  ctx_ = std::make_unique<zmq::context_t>(1);
  sock_ = std::make_unique<zmq::socket_t>(*ctx_, zmq::socket_type::req);
  // REQ sockets are strictly request-then-reply; a lost reply would otherwise hang us forever, so we
  // bound every recv with a timeout and treat expiry as a hard error (the caller decides whether to
  // retry, e.g. by reconnecting).
  sock_->set(zmq::sockopt::rcvtimeo, timeout_ms_);
  sock_->set(zmq::sockopt::sndtimeo, timeout_ms_);
  sock_->set(zmq::sockopt::linger, 0);
  sock_->connect(endpoint_);
}

URLabClient::~URLabClient() = default;

msgpack::object_handle URLabClient::call(const std::string & op,
                                         const msgpack::sbuffer & extra_fields_packed,
                                         const std::string & expect_reply_op)
{
  // Build the full request: {"op": op, "session_id": ..., <extra fields from extra_fields_packed>}
  // extra_fields_packed is expected to contain a single packed msgpack map (possibly empty); we merge
  // it with "op" (and "session_id" if we have one) into the outgoing request map.
  msgpack::sbuffer request_buf;
  msgpack::packer<msgpack::sbuffer> pk(request_buf);

  msgpack::object_handle extra_handle;
  msgpack::unpack(extra_handle, extra_fields_packed.data(), extra_fields_packed.size());
  const auto & extra = extra_handle.get();
  uint32_t extra_size = (extra.type == msgpack::type::MAP) ? extra.via.map.size : 0;

  bool has_session_field = false;
  if(extra.type == msgpack::type::MAP)
  {
    for(uint32_t i = 0; i < extra.via.map.size; ++i)
    {
      if(as_string(extra.via.map.ptr[i].key) == "session_id")
      {
        has_session_field = true;
        break;
      }
    }
  }

  uint32_t total_fields = 1 + extra_size + (!session_id_.empty() && !has_session_field ? 1 : 0);
  pk.pack_map(total_fields);
  pk.pack(std::string("op"));
  pk.pack(op);
  if(!session_id_.empty() && !has_session_field)
  {
    pk.pack(std::string("session_id"));
    pk.pack(session_id_);
  }
  if(extra.type == msgpack::type::MAP)
  {
    for(uint32_t i = 0; i < extra.via.map.size; ++i)
    {
      pk << extra.via.map.ptr[i].key;
      pk << extra.via.map.ptr[i].val;
    }
  }

  zmq::message_t request(request_buf.data(), request_buf.size());
  auto send_res = sock_->send(request, zmq::send_flags::none);
  if(!send_res.has_value())
  {
    throw URLabError("transport_error", fmt::format("Failed to send '{}' request (timeout or socket error)", op));
  }

  zmq::message_t reply;
  auto recv_res = sock_->recv(reply, zmq::recv_flags::none);
  if(!recv_res.has_value())
  {
    throw URLabError("transport_error", fmt::format("Timed out waiting for reply to '{}' (> {} ms)", op, timeout_ms_));
  }

  msgpack::object_handle reply_handle;
  try
  {
    msgpack::unpack(reply_handle, static_cast<const char *>(reply.data()), reply.size());
  }
  catch(const std::exception & e)
  {
    throw URLabError("bad_request", fmt::format("Failed to decode reply to '{}' as msgpack: {}", op, e.what()));
  }

  const auto & reply_obj = reply_handle.get();
  const auto * reply_op = find(reply_obj, "op");
  std::string reply_op_str = reply_op ? as_string(*reply_op) : "";

  if(reply_op_str == "error")
  {
    const auto * code = find(reply_obj, "code");
    const auto * message = find(reply_obj, "message");
    throw URLabError(code ? as_string(*code) : "unknown_error", message ? as_string(*message) : "(no message)");
  }

  if(reply_op_str != expect_reply_op)
  {
    throw URLabError("bad_request", fmt::format("Expected reply op '{}' to '{}' request, got '{}'", expect_reply_op, op,
                                                reply_op_str.empty() ? "<missing op field>" : reply_op_str));
  }

  return reply_handle;
}

std::vector<URLabArticulationInfo> URLabClient::parseArticulations(const msgpack::object & arr)
{
  std::vector<URLabArticulationInfo> out;
  if(arr.type != msgpack::type::ARRAY)
  {
    return out;
  }
  out.reserve(arr.via.array.size);
  for(uint32_t i = 0; i < arr.via.array.size; ++i)
  {
    const auto & a = arr.via.array.ptr[i];
    URLabArticulationInfo info;
    if(const auto * v = find(a, "prefix"))
    {
      info.prefix = as_string(*v);
    }
    if(const auto * v = find(a, "actor_id"))
    {
      info.actor_id = as_string(*v);
    }
    if(const auto * v = find(a, "default_control_mode"))
    {
      info.default_control_mode = as_string(*v);
    }
    if(const auto * names = find(a, "original_names"))
    {
      if(const auto * actuators = find(*names, "actuators"))
      {
        info.original_names["actuators"] = as_string_map(actuators);
      }
      if(const auto * joints = find(*names, "joints"))
      {
        info.original_names["joints"] = as_string_map(joints);
      }
    }
    out.push_back(std::move(info));
  }
  return out;
}

URLabStepResult URLabClient::parseStepResult(const msgpack::object & reply)
{
  URLabStepResult result;
  if(const auto * t = find(reply, "time"))
  {
    result.time = as_double(*t);
  }
  if(const auto * s = find(reply, "step"))
  {
    result.step = as_int(*s);
  }
  const auto * per_art = find(reply, "per_articulation");
  if(per_art && per_art->type == msgpack::type::MAP)
  {
    for(uint32_t i = 0; i < per_art->via.map.size; ++i)
    {
      const auto & kv = per_art->via.map.ptr[i];
      URLabArticulationObs obs;
      if(const auto * qpos = find(kv.val, "qpos"))
      {
        obs.qpos = as_double_vector(*qpos);
      }
      if(const auto * qvel = find(kv.val, "qvel"))
      {
        obs.qvel = as_double_vector(*qvel);
      }
      if(const auto * ctrl = find(kv.val, "ctrl"))
      {
        obs.ctrl = as_double_vector(*ctrl);
      }
      if(const auto * sensors = find(kv.val, "sensors"))
      {
        obs.sensors = as_named_vector_map(sensors);
      }
      result.per_articulation[as_string(kv.key)] = std::move(obs);
    }
  }
  return result;
}

URLabHandshake URLabClient::hello()
{
  msgpack::sbuffer fields;
  msgpack::packer<msgpack::sbuffer> pk(fields);
  pk.pack_map(3);
  pk.pack(std::string("observations"));
  pk.pack(std::string("standard"));
  pk.pack(std::string("encoding"));
  pk.pack(std::string("msgpack"));
  pk.pack(std::string("include_assets"));
  pk.pack(false);

  auto reply_handle = call("hello", fields, "hello_ok");
  const auto & reply = reply_handle.get();

  URLabHandshake hs;
  if(const auto * v = find(reply, "session_id"))
  {
    hs.session_id = as_string(*v);
  }
  if(const auto * v = find(reply, "urlab_version"))
  {
    hs.urlab_version = as_string(*v);
  }
  if(const auto * v = find(reply, "mujoco_version"))
  {
    hs.mujoco_version = as_string(*v);
  }
  if(const auto * v = find(reply, "mujoco_version_int"))
  {
    hs.mujoco_version_int = as_int(*v);
  }
  if(const auto * v = find(reply, "manager_present"))
  {
    hs.manager_present = as_bool(*v);
  }
  if(const auto * v = find(reply, "mjb"))
  {
    if(v->type == msgpack::type::BIN)
    {
      hs.mjb.assign(v->via.bin.ptr, v->via.bin.size);
    }
    else if(v->type == msgpack::type::STR)
    {
      // JSON-fallback sessions ship base64 alongside; msgpack sessions should always hit BIN above, but
      // guard against a misconfigured server replying as STR.
      hs.mjb.assign(v->via.str.ptr, v->via.str.size);
    }
  }
  if(const auto * v = find(reply, "articulations"))
  {
    hs.articulations = parseArticulations(*v);
  }

  if(!hs.session_id.empty())
  {
    session_id_ = hs.session_id;
  }
  return hs;
}

URLabHandshake URLabClient::beginPIE(double timeout_s)
{
  if(session_id_.empty())
  {
    throw URLabError("not_ready", "beginPIE() called before a successful hello()");
  }

  msgpack::sbuffer fields;
  msgpack::packer<msgpack::sbuffer> pk(fields);
  pk.pack_map(1);
  pk.pack(std::string("timeout_s"));
  pk.pack(timeout_s);

  auto reply_handle = call("begin_pie", fields, "begin_pie_ok");
  const auto & reply = reply_handle.get();

  std::string state;
  if(const auto * v = find(reply, "state"))
  {
    state = as_string(*v);
  }
  if(state == "compile_failed")
  {
    const auto * err = find(reply, "compile_error");
    throw URLabError("compile_failed", err ? as_string(*err) : "(no compile_error provided)");
  }
  if(state == "timeout")
  {
    throw URLabError("step_timeout", fmt::format("begin_pie did not reach 'ready' within {} s", timeout_s));
  }
  if(state != "ready")
  {
    // Already running (manager_present was true at hello time): nothing to absorb, current state stands.
    URLabHandshake hs;
    hs.session_id = session_id_;
    hs.manager_present = true;
    return hs;
  }

  const auto * payload = find(reply, "handshake_payload");
  if(!payload)
  {
    throw URLabError("not_ready", "begin_pie reached 'ready' but provided no handshake_payload");
  }

  URLabHandshake hs;
  if(const auto * v = find(*payload, "session_id"))
  {
    hs.session_id = as_string(*v);
  }
  if(const auto * v = find(*payload, "urlab_version"))
  {
    hs.urlab_version = as_string(*v);
  }
  if(const auto * v = find(*payload, "mujoco_version"))
  {
    hs.mujoco_version = as_string(*v);
  }
  if(const auto * v = find(*payload, "mujoco_version_int"))
  {
    hs.mujoco_version_int = as_int(*v);
  }
  hs.manager_present = true;
  if(const auto * v = find(*payload, "mjb"))
  {
    if(v->type == msgpack::type::BIN)
    {
      hs.mjb.assign(v->via.bin.ptr, v->via.bin.size);
    }
  }
  if(const auto * v = find(*payload, "articulations"))
  {
    hs.articulations = parseArticulations(*v);
  }
  if(!hs.session_id.empty())
  {
    session_id_ = hs.session_id;
  }
  return hs;
}

URLabStepResult URLabClient::step(const std::map<std::string, std::map<std::string, double>> & per_articulation_ctrl,
                                  size_t n_steps)
{
  msgpack::sbuffer fields;
  msgpack::packer<msgpack::sbuffer> pk(fields);
  pk.pack_map(3);
  pk.pack(std::string("n_steps"));
  pk.pack(static_cast<uint64_t>(n_steps));
  pk.pack(std::string("observations"));
  pk.pack(std::string("standard"));
  pk.pack(std::string("per_articulation"));
  pk.pack_map(static_cast<uint32_t>(per_articulation_ctrl.size()));
  for(const auto & [prefix, ctrl_map] : per_articulation_ctrl)
  {
    pk.pack(prefix);
    pk.pack_map(2);
    pk.pack(std::string("ctrl_map"));
    pack_double_map(pk, ctrl_map);
    // Force "raw": mc_rtc is the sole source of actuator commands. If we let articulations fall back to
    // their handshake default_control_mode ("ue_controller"), URLab would run its own PD on top of (or
    // instead of) what mc_mujoco already computed.
    pk.pack(std::string("control_mode"));
    pk.pack(std::string("raw"));
  }

  auto reply_handle = call("step", fields, "step_ok");
  return parseStepResult(reply_handle.get());
}

URLabStepResult URLabClient::reset(const std::string & keyframe_name,
                                   const std::map<std::string, std::map<std::string, double>> & per_articulation_qpos)
{
  msgpack::sbuffer fields;
  msgpack::packer<msgpack::sbuffer> pk(fields);
  uint32_t n_fields = (keyframe_name.empty() ? 0 : 1) + (per_articulation_qpos.empty() ? 0 : 1);
  pk.pack_map(n_fields);
  if(!keyframe_name.empty())
  {
    pk.pack(std::string("keyframe_name"));
    pk.pack(keyframe_name);
  }
  if(!per_articulation_qpos.empty())
  {
    pk.pack(std::string("per_articulation_qpos"));
    pk.pack_map(static_cast<uint32_t>(per_articulation_qpos.size()));
    for(const auto & [prefix, qpos_map] : per_articulation_qpos)
    {
      pk.pack(prefix);
      pack_double_map(pk, qpos_map);
    }
  }

  auto reply_handle = call("reset", fields, "reset_ok");
  return parseStepResult(reply_handle.get());
}

bool URLabClient::configureController(const std::string & articulation,
                                      const std::map<std::string, double> & kp,
                                      const std::map<std::string, double> & kd)
{
  try
  {
    msgpack::sbuffer fields;
    msgpack::packer<msgpack::sbuffer> pk(fields);
    pk.pack_map(2);
    pk.pack(std::string("articulation"));
    pk.pack(articulation);
    pk.pack(std::string("params"));
    pk.pack_map(2);
    pk.pack(std::string("kp"));
    pack_double_map(pk, kp);
    pk.pack(std::string("kv"));
    pack_double_map(pk, kd);

    call("configure_controller", fields, "configure_controller_ok");
    return true;
  }
  catch(const URLabError & e)
  {
    mc_rtc::log::warning("[mc_mujoco] configure_controller for {} failed: {}", articulation, e.what());
    return false;
  }
}

} // namespace mc_mujoco
