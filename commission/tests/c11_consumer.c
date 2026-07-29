#include "openarm_commission.h"

#include <stdint.h>
#include <string.h>

_Static_assert(OA_COMMISSION_ABI_V1 == UINT32_C(1), "unexpected ABI version");
_Static_assert(sizeof(((oa_commission_mapping_patch *)0)->a) == sizeof(double),
               "mapping must use double radians");

int main(void) {
    oa_commission_manual_options options;
    oa_commission_manual_session *session = 0;
    oa_commission_manual_report report;

    memset(&options, 0, sizeof(options));
    options.struct_size = (uint32_t)sizeof(options);
    options.abi_version = OA_COMMISSION_ABI_V1;
    options.side = OA_COMMISSION_LEFT;
    options.joint = 0U;
    options.expected_revision = UINT64_C(1);
    options.reference_count = 1U;
    options.known_sign = 1;
    options.minimum_samples = 2U;
    options.maximum_sample_age_ns = UINT64_C(1000000);
    options.stability_dwell_ns = UINT64_C(1000);
    options.maximum_position_spread_rad = 0.001;
    options.maximum_abs_velocity_rad_s = 0.01;
    options.minimum_reference_separation_rad = 0.1;
    options.maximum_scale_error = 0.01;
    (void)strcpy(options.motor_serial, "C11-CONSUMER");

    if (oa_commission_manual_create(&options, &session) != OA_COMMISSION_OK ||
        session == 0) {
        return 1;
    }
    memset(&report, 0, sizeof(report));
    report.struct_size = (uint32_t)sizeof(report);
    report.abi_version = OA_COMMISSION_ABI_V1;
    if (oa_commission_manual_get_report(session, &report) != OA_COMMISSION_OK ||
        report.state != OA_MANUAL_COLLECT_REFERENCE_1) {
        oa_commission_manual_destroy(session);
        return 2;
    }
    oa_commission_manual_destroy(session);
    return 0;
}
