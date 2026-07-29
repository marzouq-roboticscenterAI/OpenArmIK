/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_internal.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>
#include <string_view>

namespace openarm::runtime {

Registry<oa_runtime_manifest, ManifestData, kManifestTag> &manifests =
    *new Registry<oa_runtime_manifest, ManifestData, kManifestTag>();
Registry<oa_runtime, RuntimeData, kRuntimeTag> &runtimes =
    *new Registry<oa_runtime, RuntimeData, kRuntimeTag>();
Registry<oa_runtime_inventory, InventoryData, kInventoryTag> &inventories =
    *new Registry<oa_runtime_inventory, InventoryData, kInventoryTag>();
Registry<oa_runtime_calibration, CalibrationData, kCalibrationTag> &calibrations =
    *new Registry<oa_runtime_calibration, CalibrationData, kCalibrationTag>();
Registry<oa_runtime_plan, PlanData, kPlanTag> &plans =
    *new Registry<oa_runtime_plan, PlanData, kPlanTag>();
Registry<oa_runtime_persistence_authority, PersistenceAuthorityData, kPersistenceTag>
    &persistence_authorities =
        *new Registry<oa_runtime_persistence_authority, PersistenceAuthorityData,
                      kPersistenceTag>();

namespace {

constexpr std::array<double, 7> kLeftLower{
    -3.490659, -3.3161253267948965, -1.570796, 0.0,
    -1.570796, -0.785398, -1.570796};
constexpr std::array<double, 7> kLeftUpper{
    1.396263, 0.17453267320510335, 1.570796, 2.443461,
    1.570796, 0.785398, 1.570796};
constexpr std::array<double, 7> kRightLower{
    -1.396263, -0.17453267320510335, -1.570796, 0.0,
    -1.570796, -0.785398, -1.570796};
constexpr std::array<double, 7> kRightUpper{
    3.490659, 3.3161253267948965, 1.570796, 2.443461,
    1.570796, 0.785398, 1.570796};

template <typename T>
void append_number(std::string &out, T value) {
    std::array<char, 64> buffer{};
    std::to_chars_result result{};
    if constexpr (std::is_floating_point_v<T>) {
        result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value,
                               std::chars_format::general,
                               std::numeric_limits<T>::max_digits10);
    } else {
        result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    }
    if (result.ec != std::errc{}) {
        throw std::bad_alloc();
    }
    out.append(buffer.data(), result.ptr);
}

template <typename T>
bool parse_number(std::string_view text, T &value) {
    if (text.empty()) {
        return false;
    }
    const char *const begin = text.data();
    const char *const end = begin + text.size();
    std::from_chars_result result{};
    if constexpr (std::is_floating_point_v<T>) {
        result = std::from_chars(begin, end, value, std::chars_format::general);
        return result.ec == std::errc{} && result.ptr == end && std::isfinite(value);
    } else {
        result = std::from_chars(begin, end, value);
        return result.ec == std::errc{} && result.ptr == end;
    }
}

bool safe_text(std::string_view text, std::size_t capacity, bool allow_empty = false) {
    if ((!allow_empty && text.empty()) || text.size() >= capacity) {
        return false;
    }
    return std::all_of(text.begin(), text.end(), [](const char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
               (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.' || c == ':';
    });
}

std::vector<std::string_view> split(std::string_view line) {
    std::vector<std::string_view> fields;
    std::size_t begin = 0U;
    for (;;) {
        const std::size_t delimiter = line.find('|', begin);
        fields.push_back(line.substr(begin, delimiter == std::string_view::npos
                                               ? delimiter
                                               : delimiter - begin));
        if (delimiter == std::string_view::npos) {
            return fields;
        }
        begin = delimiter + 1U;
    }
}

std::size_t motor_offset(std::uint32_t side, std::uint32_t joint) {
    return static_cast<std::size_t>(side) * OA_RUNTIME_DOF + joint;
}

}

oa_manifest_config virtual_config() {
    oa_manifest_config config{};
    control_init(config);
    config.manifest_revision = 1U;
    config.model_revision = 1U;
    for (std::size_t side = 0U; side < 2U; ++side) {
        control_init(config.arm[side]);
        std::snprintf(config.arm[side].bus_name, sizeof(config.arm[side].bus_name),
                      "virtual%zu", side);
        for (std::size_t joint = 0U; joint < 7U; ++joint) {
            oa_motor_config &motor = config.arm[side].motor[joint];
            control_init(motor);
            motor.motor_type = joint < 2U ? OA_MOTOR_DM8009
                               : joint < 4U ? OA_MOTOR_DM4340
                                            : OA_MOTOR_DM4310;
            motor.joint_index = static_cast<std::uint32_t>(joint);
            motor.send_id = static_cast<std::uint32_t>(joint + 1U);
            motor.receive_id = static_cast<std::uint32_t>(joint + 0x11U);
            motor.embedded_motor_id = static_cast<std::uint32_t>(joint + 1U);
            motor.control_mode = 1U;
            motor.bitrate = 1000000U;
            motor.timeout_ticks = 1000U;
            motor.hardware_version = 1U;
            motor.software_version = 1U;
            motor.firmware_subversion = 1U;
            motor.q_scale = ((side + joint) & 1U) == 0U ? 1.0 : -1.0;
            motor.q_offset_rad = 0.125;
            motor.lower_rad = side == 0U ? kLeftLower[joint] : kRightLower[joint];
            motor.upper_rad = side == 0U ? kLeftUpper[joint] : kRightUpper[joint];
            motor.max_velocity_rad_s = 1.0;
            motor.max_acceleration_rad_s2 = 2.0;
            motor.max_jerk_rad_s3 = 10.0;
            motor.pmax_rad = 12.5;
            if (motor.motor_type == OA_MOTOR_DM8009) {
                motor.vmax_rad_s = 45.0;
                motor.tmax_nm = 54.0;
                motor.gear_ratio = 9.0;
            } else if (motor.motor_type == OA_MOTOR_DM4340) {
                motor.vmax_rad_s = 10.0;
                motor.tmax_nm = 28.0;
                motor.gear_ratio = 40.0;
            } else {
                motor.vmax_rad_s = 30.0;
                motor.tmax_nm = 10.0;
                motor.gear_ratio = 10.0;
            }
            motor.direction = motor.q_scale > 0.0 ? 1 : -1;
            std::snprintf(motor.serial, sizeof(motor.serial), "VIRTUAL-%zu-%zu", side,
                          joint);
            std::snprintf(motor.joint_name, sizeof(motor.joint_name),
                          "openarm_%s_joint%zu", side == 0U ? "left" : "right",
                          joint + 1U);
        }
    }
    return config;
}

bool validate_manifest_data(const ManifestData &manifest) {
    if (manifest.config.manifest_revision == 0U || manifest.config.model_revision == 0U ||
        manifest.state < OA_RUNTIME_MANIFEST_DRAFT ||
        manifest.state > OA_RUNTIME_MANIFEST_ARMABLE ||
        (manifest.intended_backend != OA_RUNTIME_BACKEND_VIRTUAL &&
         manifest.intended_backend != OA_RUNTIME_BACKEND_SOCKETCAN_QUERY &&
         manifest.intended_backend != OA_RUNTIME_BACKEND_OFFLINE)) {
        return false;
    }
    oa_manifest *validated = nullptr;
    if (oa_manifest_create(&manifest.config, &validated) != OA_CONTROL_OK) {
        return false;
    }
    oa_manifest_destroy(validated);
    for (std::size_t i = 0U; i < OA_RUNTIME_MOTORS; ++i) {
        if (!safe_text(manifest.evidence_record[i], OA_RUNTIME_TEXT_CAPACITY)) {
            return false;
        }
    }
    if (!manifest.inventory_fingerprint.empty() &&
        (manifest.inventory_fingerprint.size() != 64U ||
         !std::all_of(manifest.inventory_fingerprint.begin(),
                      manifest.inventory_fingerprint.end(), [](const char c) {
                          return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
                      }))) {
        return false;
    }
    return true;
}

std::shared_ptr<ManifestData> make_virtual_manifest() {
    auto manifest = std::make_shared<ManifestData>();
    manifest->config = virtual_config();
    manifest->state = OA_RUNTIME_MANIFEST_ARMABLE;
    manifest->intended_backend = OA_RUNTIME_BACKEND_VIRTUAL;
    manifest->inventory_revision = 1U;
    for (std::size_t i = 0U; i < OA_RUNTIME_MOTORS; ++i) {
        manifest->evidence_kind[i] = OA_EVIDENCE_SIMULATION_ONLY;
        manifest->evidence_revision[i] = 1U;
        manifest->evidence_record[i] = "builtin_virtual_exact";
    }
    const std::string preliminary = manifest_canonical(*manifest, false);
    manifest->inventory_fingerprint = sha256_hex(preliminary);
    manifest->content_digest = sha256_hex(manifest_canonical(*manifest, false));
    return manifest;
}

std::string manifest_canonical(const ManifestData &manifest, bool include_digest) {
    std::string out;
    out.reserve(12288U);
    out += "OPENARM_RUNTIME_MANIFEST|1\nsummary|";
    append_number(out, manifest.state);
    out += '|';
    append_number(out, manifest.intended_backend);
    out += '|';
    append_number(out, manifest.config.manifest_revision);
    out += '|';
    append_number(out, manifest.config.model_revision);
    out += '|';
    append_number(out, manifest.inventory_revision);
    out += '|';
    out += manifest.inventory_fingerprint.empty() ? "-" : manifest.inventory_fingerprint;
    out += '\n';
    for (std::uint32_t side = 0U; side < OA_RUNTIME_ARMS; ++side) {
        for (std::uint32_t joint = 0U; joint < OA_RUNTIME_DOF; ++joint) {
            const oa_motor_config &m = manifest.config.arm[side].motor[joint];
            const std::size_t index = motor_offset(side, joint);
            out += "motor|";
            const auto field = [&out](const auto value) {
                append_number(out, value);
                out += '|';
            };
            field(side); field(joint); field(m.motor_type); field(m.send_id);
            field(m.receive_id); field(m.embedded_motor_id); field(m.control_mode);
            field(m.bitrate); field(m.timeout_ticks); field(m.hardware_version);
            field(m.software_version); field(m.firmware_subversion); field(m.q_scale);
            field(m.q_offset_rad); field(m.lower_rad); field(m.upper_rad);
            field(m.max_velocity_rad_s); field(m.max_acceleration_rad_s2);
            field(m.max_jerk_rad_s3); field(m.pmax_rad); field(m.vmax_rad_s);
            field(m.tmax_nm); field(m.gear_ratio); field(m.direction);
            out += m.serial;
            out += '|';
            out += m.joint_name;
            out += '|';
            field(manifest.evidence_kind[index]);
            field(manifest.evidence_revision[index]);
            out += manifest.evidence_record[index];
            out += '\n';
        }
    }
    if (include_digest) {
        out += "sha256|";
        out += manifest.content_digest.empty() ? sha256_hex(out) : manifest.content_digest;
        out += '\n';
    }
    return out;
}

oa_runtime_status parse_manifest(const std::string &text,
                                 std::shared_ptr<ManifestData> &out) {
    if (text.empty() || text.size() > 65536U || text.back() != '\n' ||
        std::find(text.begin(), text.end(), '\0') != text.end()) {
        return OA_RUNTIME_ECORRUPT;
    }
    std::vector<std::string_view> lines;
    std::size_t begin = 0U;
    while (begin < text.size()) {
        const std::size_t end = text.find('\n', begin);
        if (end == std::string::npos || end - begin > 2048U) {
            return OA_RUNTIME_ECORRUPT;
        }
        lines.emplace_back(text.data() + begin, end - begin);
        begin = end + 1U;
    }
    if ((lines.size() != 17U && lines.size() != 18U) ||
        lines[0] != "OPENARM_RUNTIME_MANIFEST|1") {
        return OA_RUNTIME_ECORRUPT;
    }
    const std::size_t digest_marker = text.find("\nsha256|");
    if (digest_marker == std::string::npos) {
        return OA_RUNTIME_ECORRUPT;
    }
    const std::size_t digest_line_offset = digest_marker + 1U;
    const auto digest_fields = split(lines[16U]);
    if (digest_fields.size() != 2U || digest_fields[0] != "sha256" ||
        digest_fields[1].size() != 64U ||
        sha256_hex(reinterpret_cast<const std::uint8_t *>(text.data()), digest_line_offset) !=
            digest_fields[1]) {
        return OA_RUNTIME_ECORRUPT;
    }
    auto result = std::make_shared<ManifestData>();
    control_init(result->config);
    const auto summary = split(lines[1]);
    if (summary.size() != 7U || summary[0] != "summary" ||
        !parse_number(summary[1], result->state) ||
        !parse_number(summary[2], result->intended_backend) ||
        !parse_number(summary[3], result->config.manifest_revision) ||
        !parse_number(summary[4], result->config.model_revision) ||
        !parse_number(summary[5], result->inventory_revision)) {
        return OA_RUNTIME_ECORRUPT;
    }
    result->inventory_fingerprint = summary[6] == "-" ? "" : std::string(summary[6]);
    for (std::uint32_t side = 0U; side < OA_RUNTIME_ARMS; ++side) {
        control_init(result->config.arm[side]);
        std::snprintf(result->config.arm[side].bus_name,
                      sizeof(result->config.arm[side].bus_name),
                      result->intended_backend == OA_RUNTIME_BACKEND_VIRTUAL
                          ? "virtual%u"
                          : "can%u",
                      side);
        for (std::uint32_t joint = 0U; joint < OA_RUNTIME_DOF; ++joint) {
            const auto fields = split(lines[2U + motor_offset(side, joint)]);
            if (fields.size() != 30U || fields[0] != "motor") {
                return OA_RUNTIME_ECORRUPT;
            }
            oa_motor_config &m = result->config.arm[side].motor[joint];
            control_init(m);
            std::uint32_t parsed_side = 0U;
            std::uint32_t parsed_joint = 0U;
            bool valid = parse_number(fields[1], parsed_side) &&
                         parse_number(fields[2], parsed_joint) && parsed_side == side &&
                         parsed_joint == joint && parse_number(fields[3], m.motor_type) &&
                         parse_number(fields[4], m.send_id) &&
                         parse_number(fields[5], m.receive_id) &&
                         parse_number(fields[6], m.embedded_motor_id) &&
                         parse_number(fields[7], m.control_mode) &&
                         parse_number(fields[8], m.bitrate) &&
                         parse_number(fields[9], m.timeout_ticks) &&
                         parse_number(fields[10], m.hardware_version) &&
                         parse_number(fields[11], m.software_version) &&
                         parse_number(fields[12], m.firmware_subversion) &&
                         parse_number(fields[13], m.q_scale) &&
                         parse_number(fields[14], m.q_offset_rad) &&
                         parse_number(fields[15], m.lower_rad) &&
                         parse_number(fields[16], m.upper_rad) &&
                         parse_number(fields[17], m.max_velocity_rad_s) &&
                         parse_number(fields[18], m.max_acceleration_rad_s2) &&
                         parse_number(fields[19], m.max_jerk_rad_s3) &&
                         parse_number(fields[20], m.pmax_rad) &&
                         parse_number(fields[21], m.vmax_rad_s) &&
                         parse_number(fields[22], m.tmax_nm) &&
                         parse_number(fields[23], m.gear_ratio) &&
                         parse_number(fields[24], m.direction) &&
                         safe_text(fields[25], sizeof(m.serial)) &&
                         safe_text(fields[26], sizeof(m.joint_name));
            const std::size_t index = motor_offset(side, joint);
            valid = valid && parse_number(fields[27], result->evidence_kind[index]) &&
                    parse_number(fields[28], result->evidence_revision[index]) &&
                    safe_text(fields[29], OA_RUNTIME_TEXT_CAPACITY);
            if (!valid) {
                return OA_RUNTIME_ECORRUPT;
            }
            m.joint_index = joint;
            std::memcpy(m.serial, fields[25].data(), fields[25].size());
            std::memcpy(m.joint_name, fields[26].data(), fields[26].size());
            result->evidence_record[index] = fields[29];
        }
    }
    result->content_digest = digest_fields[1];
    if (lines.size() == 18U) {
        const auto authentication = split(lines[17U]);
        if (authentication.size() != 3U || authentication[0] != "hmac-sha256" ||
            !safe_text(authentication[1], OA_RUNTIME_TEXT_CAPACITY) ||
            authentication[2].size() != 64U ||
            !std::all_of(authentication[2].begin(), authentication[2].end(),
                         [](const char c) {
                             return (c >= '0' && c <= '9') ||
                                    (c >= 'a' && c <= 'f');
                         })) {
            return OA_RUNTIME_ECORRUPT;
        }
        result->integrity_kind = OA_RUNTIME_INTEGRITY_HMAC_SHA256;
        result->authentication_key_id = authentication[1];
        result->authentication_tag = authentication[2];
    }
    if (!validate_manifest_data(*result)) {
        return OA_RUNTIME_ECORRUPT;
    }
    out = std::move(result);
    return OA_RUNTIME_OK;
}

std::shared_ptr<ManifestData> apply_patch(const ManifestData &base,
                                         const oa_commission_mapping_patch &patch,
                                         oa_runtime_manifest_preview &preview) {
    runtime_init(preview);
    preview.base_revision = base.config.manifest_revision;
    preview.result_revision = patch.replacement_revision;
    preview.validation_status = OA_RUNTIME_EINVAL;
    if (patch.struct_size < sizeof(patch) || patch.abi_version != OA_COMMISSION_ABI_V1 ||
        patch.expected_revision != base.config.manifest_revision ||
        patch.replacement_revision <= patch.expected_revision || patch.side >= 2U ||
        patch.joint >= 7U || (patch.a != 1.0 && patch.a != -1.0) ||
        !std::isfinite(patch.b_rad)) {
        return {};
    }
    const std::size_t index = motor_offset(patch.side, patch.joint);
    const oa_motor_config &source = base.config.arm[patch.side].motor[patch.joint];
    if (std::strncmp(source.serial, patch.motor_serial, sizeof(source.serial)) != 0 ||
        (base.intended_backend != OA_RUNTIME_BACKEND_VIRTUAL &&
         patch.evidence_kind == OA_EVIDENCE_SIMULATION_ONLY) ||
        (patch.evidence_kind != OA_EVIDENCE_MANUAL_FIXTURE &&
         patch.evidence_kind != OA_EVIDENCE_HARDWARE_QUALIFIED &&
         patch.evidence_kind != OA_EVIDENCE_SIMULATION_ONLY) ||
        !safe_text(patch.evidence_record, OA_COMMISSION_TEXT_CAPACITY)) {
        preview.validation_status = OA_RUNTIME_EIDENTITY;
        return {};
    }
    auto result = std::make_shared<ManifestData>(base);
    result->integrity_kind = OA_RUNTIME_INTEGRITY_UNKEYED_SHA256;
    result->authenticated = false;
    result->authentication_key_id.clear();
    result->authentication_tag.clear();
    result->loaded_from_file = false;
    oa_motor_config &motor = result->config.arm[patch.side].motor[patch.joint];
    motor.q_scale = patch.a;
    motor.q_offset_rad = patch.b_rad;
    motor.direction = patch.a > 0.0 ? 1 : -1;
    result->config.manifest_revision = patch.replacement_revision;
    result->state = base.intended_backend == OA_RUNTIME_BACKEND_VIRTUAL
                        ? OA_RUNTIME_MANIFEST_ARMABLE
                        : OA_RUNTIME_MANIFEST_LOCALLY_VALID;
    result->evidence_kind[index] = patch.evidence_kind;
    result->evidence_revision[index] = patch.fixture_revision != 0U
                                           ? patch.fixture_revision
                                           : patch.qualification_revision;
    result->evidence_record[index] = patch.evidence_record;
    if (!validate_manifest_data(*result)) {
        preview.validation_status = OA_RUNTIME_EINVAL;
        return {};
    }
    result->content_digest = sha256_hex(manifest_canonical(*result, false));
    preview.valid = 1U;
    preview.changed_motor_mask = UINT32_C(1) << index;
    preview.mapping_change_mask = preview.changed_motor_mask;
    preview.would_be_armable = result->state == OA_RUNTIME_MANIFEST_ARMABLE ? 1U : 0U;
    preview.validation_status = OA_RUNTIME_OK;
    return result;
}

}

extern "C" oa_runtime_status
oa_runtime_manifest_create_virtual(oa_runtime_manifest **out_manifest) {
    if (out_manifest == nullptr) {
        return OA_RUNTIME_EINVAL;
    }
    *out_manifest = nullptr;
    try {
        const auto manifest = openarm::runtime::make_virtual_manifest();
        oa_runtime_manifest *const handle = openarm::runtime::manifests.insert(manifest);
        if (handle == nullptr) {
            return OA_RUNTIME_ENOMEM;
        }
        *out_manifest = handle;
        return OA_RUNTIME_OK;
    } catch (...) {
        return OA_RUNTIME_ENOMEM;
    }
}

extern "C" oa_runtime_status oa_runtime_manifest_get_summary(
    const oa_runtime_manifest *manifest, oa_runtime_manifest_summary *out_summary) {
    if (!openarm::runtime::output_valid(out_summary)) {
        return OA_RUNTIME_EABI;
    }
    const auto pinned = openarm::runtime::manifests.pin(manifest);
    if (!pinned) {
        return OA_RUNTIME_EINVAL;
    }
    oa_runtime_manifest_summary result{};
    openarm::runtime::runtime_init(result);
    result.state = pinned->state;
    result.intended_backend = pinned->intended_backend;
    result.manifest_revision = pinned->config.manifest_revision;
    result.model_revision = pinned->config.model_revision;
    result.inventory_revision = pinned->inventory_revision;
    result.integrity_kind = pinned->integrity_kind;
    result.authenticated = pinned->authenticated ? 1U : 0U;
    std::snprintf(result.inventory_fingerprint_sha256,
                  sizeof(result.inventory_fingerprint_sha256), "%s",
                  pinned->inventory_fingerprint.c_str());
    std::snprintf(result.content_sha256, sizeof(result.content_sha256), "%s",
                  pinned->content_digest.c_str());
    std::snprintf(result.authentication_key_id, sizeof(result.authentication_key_id), "%s",
                  pinned->authentication_key_id.c_str());
    *out_summary = result;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_manifest_get_motor(
    const oa_runtime_manifest *manifest, std::uint32_t side, std::uint32_t joint,
    oa_runtime_motor_manifest *out_motor) {
    if (!openarm::runtime::output_valid(out_motor)) {
        return OA_RUNTIME_EABI;
    }
    const auto pinned = openarm::runtime::manifests.pin(manifest);
    if (!pinned || side >= 2U || joint >= 7U) {
        return OA_RUNTIME_EINVAL;
    }
    oa_runtime_motor_manifest result{};
    openarm::runtime::runtime_init(result);
    const oa_motor_config &m = pinned->config.arm[side].motor[joint];
    result.side = side; result.joint = joint; result.motor_type = m.motor_type;
    result.send_id = m.send_id; result.receive_id = m.receive_id;
    result.embedded_motor_id = m.embedded_motor_id; result.control_mode = m.control_mode;
    result.bitrate = m.bitrate; result.timeout_ticks = m.timeout_ticks;
    result.hardware_version = m.hardware_version; result.software_version = m.software_version;
    result.firmware_subversion = m.firmware_subversion; result.q_scale = m.q_scale;
    result.q_offset_rad = m.q_offset_rad; result.lower_rad = m.lower_rad;
    result.upper_rad = m.upper_rad; result.maximum_velocity_rad_s = m.max_velocity_rad_s;
    result.maximum_acceleration_rad_s2 = m.max_acceleration_rad_s2;
    result.maximum_jerk_rad_s3 = m.max_jerk_rad_s3;
    result.protocol_pmax_rad = m.pmax_rad; result.protocol_vmax_rad_s = m.vmax_rad_s;
    result.protocol_tmax_nm = m.tmax_nm; result.gear_ratio_metadata = m.gear_ratio;
    result.direction = m.direction;
    const std::size_t index = static_cast<std::size_t>(side) * 7U + joint;
    result.evidence_kind = pinned->evidence_kind[index];
    result.evidence_revision = pinned->evidence_revision[index];
    std::snprintf(result.serial, sizeof(result.serial), "%s", m.serial);
    std::snprintf(result.joint_name, sizeof(result.joint_name), "%s", m.joint_name);
    std::snprintf(result.evidence_record, sizeof(result.evidence_record), "%s",
                  pinned->evidence_record[index].c_str());
    *out_motor = result;
    return OA_RUNTIME_OK;
}

extern "C" void oa_runtime_manifest_destroy(oa_runtime_manifest *manifest) {
    openarm::runtime::manifests.erase(manifest);
}
