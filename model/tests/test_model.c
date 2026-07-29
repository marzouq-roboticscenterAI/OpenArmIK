/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_model.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned state = 0x42u;
static double random_unit(void) { state = state * 1664525u + 1013904223u; return (double)(state >> 8) / 16777216.0; }
static int failures;
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); ++failures; } } while (0)
static double distance3(const double a[3], const double b[3]) { double x=a[0]-b[0],y=a[1]-b[1],z=a[2]-b[2]; return sqrt(x*x+y*y+z*z); }
static void defaults(oa_ik_options *o, const double seed[OA_DOF]) {
    size_t i; memset(o, 0, sizeof(*o)); o->abi_version=OA_MODEL_ABI_VERSION; o->struct_size=sizeof(*o);
    o->position_tolerance_m=1e-5; o->max_joint_step_rad=0.15; o->damping_min=1e-4; o->damping_max=0.3; o->max_iterations=500;
    for(i=0;i<OA_DOF;++i) { o->seed[i]=seed[i]; o->posture[i]=seed[i]; o->posture_weight[i]=1.0; }
}
static void test_metadata_and_bounds(const oa_model *m) {
    double lo, hi; size_t i; CHECK(oa_model_id(m) != NULL); CHECK(strlen(oa_model_data_sha256(m)) == 64); CHECK(strlen(oa_model_source_sha256(m)) == 64);
    CHECK(strstr(oa_model_provenance(m), "6c7b720f") != NULL); CHECK(strstr(oa_model_tip_frame(m), "hand_tcp") != NULL);
    for(i=0;i<OA_DOF;++i) { CHECK(oa_model_limits(m,i,&lo,&hi)==OA_OK); CHECK(lo<hi); CHECK(oa_model_joint_name(m,i)!=NULL); }
    CHECK(oa_model_limits(m,OA_DOF,&lo,&hi)==OA_EINVAL);
}
static void test_fk_jacobian(const oa_model *m) {
    double q[OA_DOF]={0}, qp[OA_DOF], qm[OA_DOF], p[3], pp[3], pm[3], lo, hi; oa_fk_result fk; oa_jacobian j; size_t i,r,n; const double h=1e-7;
    CHECK(oa_fk(m,q,&fk)==OA_OK); CHECK(oa_geometric_jacobian(m,q,&j)==OA_OK); p[0]=fk.hand_tcp.m[3];p[1]=fk.hand_tcp.m[7];p[2]=fk.hand_tcp.m[11]; { double link7[3]={fk.link_post[6].m[3],fk.link_post[6].m[7],fk.link_post[6].m[11]}; CHECK(fabs(distance3(p,link7)-0.186)<1e-12); }
    for(i=0;i<OA_DOF;++i) { memcpy(qp,q,sizeof(q));memcpy(qm,q,sizeof(q));qp[i]+=h;qm[i]-=h; CHECK(oa_fk(m,qp,&fk)==OA_OK);pp[0]=fk.hand_tcp.m[3];pp[1]=fk.hand_tcp.m[7];pp[2]=fk.hand_tcp.m[11]; CHECK(oa_fk(m,qm,&fk)==OA_OK);pm[0]=fk.hand_tcp.m[3];pm[1]=fk.hand_tcp.m[7];pm[2]=fk.hand_tcp.m[11]; for(r=0;r<3;++r) CHECK(fabs((pp[r]-pm[r])/(2*h)-j.value[r][i])<2e-6); CHECK(fabs(j.value[3][i]-fk.joint_axis_body[i][0])<1e-12); CHECK(fabs(j.value[4][i]-fk.joint_axis_body[i][1])<1e-12); CHECK(fabs(j.value[5][i]-fk.joint_axis_body[i][2])<1e-12); }
    q[0]=NAN; CHECK(oa_fk(m,q,&fk)==OA_ENONFINITE); CHECK(oa_geometric_jacobian(m,q,&j)==OA_ENONFINITE); (void)p;
    for (i=0;i<OA_DOF;++i) { oa_model_limits(m,i,&lo,&hi); memset(q,0,sizeof(q)); q[i]=lo; CHECK(oa_fk(m,q,&fk)==OA_OK); q[i]=hi; CHECK(oa_geometric_jacobian(m,q,&j)==OA_OK); }
    for (n=0;n<30;++n) { for(i=0;i<OA_DOF;++i){oa_model_limits(m,i,&lo,&hi);q[i]=lo+(hi-lo)*random_unit();} CHECK(oa_fk(m,q,&fk)==OA_OK); CHECK(isfinite(fk.hand_tcp.m[3]) && isfinite(fk.hand_tcp.m[7]) && isfinite(fk.hand_tcp.m[11])); }
}
static void test_ik(const oa_model *m) {
    size_t n,i; double q[OA_DOF], seed[OA_DOF], lo,hi,target[3]; oa_fk_result fk; oa_ik_options o; oa_ik_diagnostics d;
    for(n=0;n<20;++n) { for(i=0;i<OA_DOF;++i){oa_model_limits(m,i,&lo,&hi);q[i]=lo+0.1*(hi-lo)+0.8*(hi-lo)*random_unit();seed[i]=fmin(hi-1e-3,fmax(lo+1e-3,q[i]+0.05*(random_unit()-0.5)));} CHECK(oa_fk(m,q,&fk)==OA_OK);target[0]=fk.hand_tcp.m[3];target[1]=fk.hand_tcp.m[7];target[2]=fk.hand_tcp.m[11];defaults(&o,seed); CHECK(oa_ik_position(m,target,&o,&d)==OA_OK); CHECK(d.position_error_m<=o.position_tolerance_m*1.01); CHECK(d.collision_checked==0); for(i=0;i<OA_DOF;++i){oa_model_limits(m,i,&lo,&hi);CHECK(d.q[i]>=lo-1e-12 && d.q[i]<=hi+1e-12);} }
    defaults(&o,seed); target[0]=10;target[1]=10;target[2]=10; CHECK(oa_ik_position(m,target,&o,&d)!=OA_OK); CHECK(d.collision_checked==0);
    target[0]=NAN; CHECK(oa_ik_position(m,target,&o,&d)==OA_ENONFINITE); o.max_iterations=0; target[0]=0; CHECK(oa_ik_position(m,target,&o,&d)==OA_EINVAL);
    defaults(&o,seed); CHECK(oa_ik_position(NULL,target,&o,&d)==OA_EINVAL); CHECK(oa_ik_position(m,target,NULL,&d)==OA_EINVAL); o.abi_version=0; CHECK(oa_ik_position(m,target,&o,&d)==OA_EINVAL);
}
static void test_path_continuity(const oa_model *m) {
    double q[OA_DOF], previous[OA_DOF], lo,hi,target[3]; oa_fk_result fk; oa_ik_options o; oa_ik_diagnostics d; size_t i,n;
    for(i=0;i<OA_DOF;++i){oa_model_limits(m,i,&lo,&hi);q[i]=0.5*(lo+hi);previous[i]=q[i];}
    for(n=0;n<8;++n){q[0]+=0.005;CHECK(oa_fk(m,q,&fk)==OA_OK);target[0]=fk.hand_tcp.m[3];target[1]=fk.hand_tcp.m[7];target[2]=fk.hand_tcp.m[11];defaults(&o,previous);o.position_tolerance_m=1e-4;o.max_iterations=1000;CHECK(oa_ik_position(m,target,&o,&d)==OA_OK);for(i=0;i<OA_DOF;++i){CHECK(fabs(d.q[i]-previous[i])<0.25);previous[i]=d.q[i];}}
}
int main(void) {
    const oa_model *models[2]={oa_model_left_v10_bimanual(),oa_model_right_v10_bimanual()}; size_t i;
    for(i=0;i<2;++i){test_metadata_and_bounds(models[i]);test_fk_jacobian(models[i]);test_ik(models[i]);test_path_continuity(models[i]);}
    if(failures){fprintf(stderr,"%d failures\n",failures);return 1;} puts("oa model tests passed");return 0;
}
