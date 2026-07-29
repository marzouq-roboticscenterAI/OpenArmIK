#ifndef OPENARM_CALIBRATION_SESSION_HPP
#define OPENARM_CALIBRATION_SESSION_HPP

#include "openarm_commission.h"

#include <array>
#include <cstdint>
#include <string>

namespace openarm::commission {

struct SampleAccumulator {
    std::uint32_t count{0};
    std::uint64_t first_time_ns{0};
    double sum{0.0};
    double minimum{0.0};
    double maximum{0.0};

    void reset(const oa_commission_encoder_sample &sample) noexcept;
    void add(const oa_commission_encoder_sample &sample) noexcept;
    [[nodiscard]] double mean() const noexcept;
    [[nodiscard]] double spread() const noexcept;
};

class ManualCalibrationSession final {
public:
    explicit ManualCalibrationSession(const oa_commission_manual_options &options);

    oa_commission_status sample(std::uint32_t reference_index,
                                std::uint64_t now_ns,
                                const oa_commission_encoder_sample &sample) noexcept;
    oa_commission_status begin_review() noexcept;
    oa_commission_status commit(std::uint64_t replacement_revision,
                                const char *evidence_record,
                                oa_commission_mapping_patch &patch) noexcept;
    oa_commission_status abort() noexcept;
    [[nodiscard]] oa_commission_manual_report report() const noexcept;

private:
    oa_commission_manual_options options_{};
    std::array<SampleAccumulator, 2> accumulators_{};
    std::array<double, 2> means_{};
    std::array<double, 2> spreads_{};
    std::uint64_t last_feedback_seq_{0};
    std::uint64_t last_sample_time_ns_{0};
    std::uint32_t state_{OA_MANUAL_COLLECT_REFERENCE_1};
    double candidate_a_{0.0};
    double candidate_b_{0.0};

    oa_commission_status validate_sample(std::uint64_t now_ns,
                                         const oa_commission_encoder_sample &sample) const noexcept;
    oa_commission_status finish_reference(std::uint32_t reference_index) noexcept;
    oa_commission_status calculate_candidate() noexcept;
};

class RecipeCalibrationSession final {
public:
    explicit RecipeCalibrationSession(const oa_commission_recipe &recipe);

    oa_commission_status step(const oa_commission_recipe_input &input,
                              oa_commission_next_action &action) noexcept;
    oa_commission_status commit(std::uint64_t replacement_revision,
                                oa_commission_mapping_patch &patch) noexcept;
    oa_commission_status abort() noexcept;
    [[nodiscard]] oa_commission_recipe_report report() const noexcept;

private:
    oa_commission_recipe recipe_{};
    std::uint32_t state_{OA_RECIPE_PRECHECK};
    std::uint32_t abort_reason_{OA_ABORT_NONE};
    std::uint64_t last_feedback_seq_{0};
    std::uint64_t last_sample_time_ns_{0};
    std::uint64_t phase_start_ns_{0};
    std::uint64_t dwell_start_ns_{0};
    std::uint64_t last_energy_time_ns_{0};
    std::uint32_t contact_samples_{0};
    double phase_start_q_{0.0};
    double contact_sum_q_{0.0};
    double first_stop_q_{0.0};
    double second_stop_q_{0.0};
    double candidate_a_{0.0};
    double candidate_b_{0.0};
    double contact_energy_j_{0.0};

    oa_commission_status validate_input(const oa_commission_recipe_input &input) noexcept;
    oa_commission_status fail(oa_commission_status status, std::uint32_t reason) noexcept;
    oa_commission_status enforce_motion_limits(const oa_commission_recipe_input &input) noexcept;
    oa_commission_status update_energy(const oa_commission_recipe_input &input) noexcept;
    [[nodiscard]] bool contact_evidence(const oa_commission_recipe_input &input,
                                        double travel) const noexcept;
    [[nodiscard]] oa_commission_next_action action(std::uint32_t kind,
                                                   const oa_commission_recipe_input &input) const noexcept;
    [[nodiscard]] double directional_travel(double from,
                                            double to,
                                            double direction) const noexcept;
};

bool valid_header(std::uint32_t struct_size,
                  std::uint32_t abi_version,
                  std::uint32_t required_size) noexcept;
bool valid_text(const char *text, bool allow_empty = false) noexcept;

}  // namespace openarm::commission

#endif
