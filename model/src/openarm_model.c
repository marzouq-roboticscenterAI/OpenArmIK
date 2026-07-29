/* SPDX-License-Identifier: Apache-2.0 */
#include "oa_model_internal.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "generated/oa_model_data.inc"

static int finite_n(const double *v, size_t n) {
    size_t i;
    for (i = 0; i < n; ++i) if (!isfinite(v[i])) return 0;
    return 1;
}
static void identity(oa_transform *t) {
    memset(t, 0, sizeof(*t)); t->m[0] = t->m[5] = t->m[10] = t->m[15] = 1.0;
}
static void mul(oa_transform *r, const oa_transform *a, const oa_transform *b) {
    oa_transform x; size_t i, j, k;
    for (i = 0; i < 4; ++i) for (j = 0; j < 4; ++j) {
        x.m[i * 4 + j] = 0.0;
        for (k = 0; k < 4; ++k) x.m[i * 4 + j] += a->m[i * 4 + k] * b->m[k * 4 + j];
    }
    *r = x;
}
static void apply_dir(const oa_transform *t, const double v[3], double out[3]) {
    size_t i;
    for (i = 0; i < 3; ++i) out[i] = t->m[i * 4] * v[0] + t->m[i * 4 + 1] * v[1] + t->m[i * 4 + 2] * v[2];
}
static void rotation_axis(oa_transform *t, const double a[3], double q) {
    const double c = cos(q), s = sin(q), d = 1.0 - c, x = a[0], y = a[1], z = a[2];
    identity(t);
    t->m[0] = c + x*x*d; t->m[1] = x*y*d - z*s; t->m[2] = x*z*d + y*s;
    t->m[4] = y*x*d + z*s; t->m[5] = c + y*y*d; t->m[6] = y*z*d - x*s;
    t->m[8] = z*x*d - y*s; t->m[9] = z*y*d + x*s; t->m[10] = c + z*z*d;
}
static void pos(const oa_transform *t, double p[3]) { p[0] = t->m[3]; p[1] = t->m[7]; p[2] = t->m[11]; }
static double norm3(const double v[3]) { return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }

const oa_model *oa_model_left_v10_bimanual(void) { return &oa_left; }
const oa_model *oa_model_right_v10_bimanual(void) { return &oa_right; }
const char *oa_model_id(const oa_model *m) { return m ? m->id : NULL; }
const char *oa_model_provenance(const oa_model *m) { return m ? m->provenance : NULL; }
const char *oa_model_data_sha256(const oa_model *m) { return m ? m->data_sha256 : NULL; }
const char *oa_model_source_sha256(const oa_model *m) { return m ? m->source_sha256 : NULL; }
const char *oa_model_joint_name(const oa_model *m, size_t i) { return (m && i < OA_DOF) ? m->joint_name[i] : NULL; }
const char *oa_model_tip_frame(const oa_model *m) { return m ? m->tip_frame : NULL; }
oa_status oa_model_limits(const oa_model *m, size_t i, double *lo, double *hi) {
    if (!m || !lo || !hi || i >= OA_DOF) return OA_EINVAL;
    *lo = m->lower[i]; *hi = m->upper[i]; return OA_OK;
}

oa_status oa_fk(const oa_model *m, const double q[OA_DOF], oa_fk_result *out) {
    oa_transform current, rot; size_t i;
    if (!m || !q || !out) return OA_EINVAL;
    if (!finite_n(q, OA_DOF)) return OA_ENONFINITE;
    current = m->base_in_body; out->base_in_body = current;
    for (i = 0; i < OA_DOF; ++i) {
        mul(&out->joint_pre[i], &current, &m->origin[i]);
        apply_dir(&out->joint_pre[i], m->axis[i], out->joint_axis_body[i]);
        rotation_axis(&rot, m->axis[i], q[i]);
        mul(&current, &out->joint_pre[i], &rot);
        out->link_post[i] = current;
    }
    mul(&out->hand_tcp, &current, &m->tcp_in_link7);
    return OA_OK;
}

oa_status oa_geometric_jacobian(const oa_model *m, const double q[OA_DOF], oa_jacobian *out) {
    oa_fk_result fk; double p[3], o[3], d[3]; size_t i; oa_status st;
    if (!out) return OA_EINVAL;
    st = oa_fk(m, q, &fk); if (st != OA_OK) return st;
    pos(&fk.hand_tcp, p);
    for (i = 0; i < OA_DOF; ++i) {
        pos(&fk.joint_pre[i], o); d[0] = p[0]-o[0]; d[1] = p[1]-o[1]; d[2] = p[2]-o[2];
        out->value[0][i] = fk.joint_axis_body[i][1]*d[2] - fk.joint_axis_body[i][2]*d[1];
        out->value[1][i] = fk.joint_axis_body[i][2]*d[0] - fk.joint_axis_body[i][0]*d[2];
        out->value[2][i] = fk.joint_axis_body[i][0]*d[1] - fk.joint_axis_body[i][1]*d[0];
        out->value[3][i] = fk.joint_axis_body[i][0]; out->value[4][i] = fk.joint_axis_body[i][1]; out->value[5][i] = fk.joint_axis_body[i][2];
    }
    return OA_OK;
}

static int solve3(const double a_in[3][3], const double b[3], double x[3]) {
    double a[3][4]; size_t i, j, k, p; double v, pivot;
    for (i=0;i<3;++i) { for(j=0;j<3;++j) a[i][j]=a_in[i][j]; a[i][3]=b[i]; }
    for (k=0;k<3;++k) {
        p=k; for(i=k+1;i<3;++i) if(fabs(a[i][k])>fabs(a[p][k])) p=i;
        if(fabs(a[p][k]) < 1e-15) return 0;
        if(p!=k) for(j=k;j<4;++j) { v=a[k][j];a[k][j]=a[p][j];a[p][j]=v; }
        pivot=a[k][k]; for(j=k;j<4;++j) a[k][j]/=pivot;
        for(i=0;i<3;++i) if(i!=k) { v=a[i][k]; for(j=k;j<4;++j) a[i][j]-=v*a[k][j]; }
    }
    for (i = 0; i < 3; ++i) x[i] = a[i][3];
    return 1;
}
static double min_eigen_symmetric3(const double in[3][3]) {
    double a[3][3], max, c, s, t, app, aqq, apq; size_t it, p, q, k;
    memcpy(a, in, sizeof(a));
    for (it=0; it<20; ++it) {
        p=0;q=1;max=fabs(a[0][1]); if(fabs(a[0][2])>max){p=0;q=2;max=fabs(a[0][2]);} if(fabs(a[1][2])>max){p=1;q=2;max=fabs(a[1][2]);}
        if(max < 1e-14) break;
        t = 0.5 * atan2(2.0*a[p][q], a[q][q]-a[p][p]); c=cos(t);s=sin(t); app=a[p][p];aqq=a[q][q];apq=a[p][q];
        a[p][p]=c*c*app-2*c*s*apq+s*s*aqq; a[q][q]=s*s*app+2*c*s*apq+c*c*aqq; a[p][q]=a[q][p]=0.0;
        for(k=0;k<3;++k) if(k!=p && k!=q){ double akp=a[k][p], akq=a[k][q]; a[k][p]=a[p][k]=c*akp-s*akq; a[k][q]=a[q][k]=s*akp+c*akq; }
    }
    return fmax(0.0, fmin(a[0][0], fmin(a[1][1],a[2][2])));
}
static double merit(const double e[3], const double q[OA_DOF], const oa_ik_options *o) {
    double v=e[0]*e[0]+e[1]*e[1]+e[2]*e[2]; size_t i;
    for(i=0;i<OA_DOF;++i) { double d=q[i]-o->posture[i]; v += 1e-6*o->posture_weight[i]*d*d; }
    return v;
}

oa_status oa_ik_position(const oa_model *m, const double target[3], const oa_ik_options *o, oa_ik_diagnostics *out) {
    double q[OA_DOF], p[3], e[3], y[3], j[3][OA_DOF], a[3][3], pinv[OA_DOF][3], step[OA_DOF], nullv[OA_DOF], candidate[OA_DOF];
    uint32_t iter, active = 0; size_t i,k,r,c; oa_fk_result fk; oa_jacobian jac; oa_status st = OA_ENOCONVERGENCE; double sigma=0.0;
    if (!m || !target || !o || !out || o->abi_version != OA_MODEL_ABI_VERSION || o->struct_size < sizeof(*o)) return OA_EINVAL;
    memset(out, 0, sizeof(*out)); out->collision_checked = 0;
    if (!finite_n(target,3) || !finite_n(o->seed,OA_DOF) || !finite_n(o->posture,OA_DOF) || !finite_n(o->posture_weight,OA_DOF) ||
        !isfinite(o->position_tolerance_m) || !isfinite(o->max_joint_step_rad) || !isfinite(o->damping_min) || !isfinite(o->damping_max) || !isfinite(o->limit_margin_rad)) return OA_ENONFINITE;
    if (o->max_iterations == 0 || o->position_tolerance_m <= 0.0 || o->max_joint_step_rad <= 0.0 || o->damping_min < 0.0 || o->damping_max < o->damping_min || o->limit_margin_rad < 0.0) return OA_EINVAL;
    for(i=0;i<OA_DOF;++i) { if(o->posture_weight[i] <= 0.0 || o->limit_margin_rad*2.0 >= m->upper[i]-m->lower[i]) return OA_EINVAL; q[i]=fmin(m->upper[i]-o->limit_margin_rad, fmax(m->lower[i]+o->limit_margin_rad,o->seed[i])); }
    for(iter=0; iter<o->max_iterations; ++iter) {
        st=oa_fk(m,q,&fk); if(st!=OA_OK) break; pos(&fk.hand_tcp,p); for(r=0;r<3;++r)e[r]=target[r]-p[r];
        if(norm3(e) <= o->position_tolerance_m) { st=OA_OK; break; }
        st=oa_geometric_jacobian(m,q,&jac); if(st!=OA_OK) break;
        memset(a,0,sizeof(a)); for(r=0;r<3;++r) for(i=0;i<OA_DOF;++i) { j[r][i]=jac.value[r][i]; if(!(active&(1u<<i))) for(c=0;c<3;++c)a[r][c]+=j[r][i]*j[c][i]/o->posture_weight[i]; }
        sigma=sqrt(min_eigen_symmetric3(a)); { double lambda=o->damping_min; if(sigma < 0.03) lambda=o->damping_min+(o->damping_max-o->damping_min)*(0.03-sigma)/0.03; if(lambda>o->damping_max)lambda=o->damping_max; for(r=0;r<3;++r)a[r][r]+=lambda*lambda; }
        if(!solve3(a,e,y)) { st=OA_ESINGULAR; break; }
        memset(step,0,sizeof(step)); memset(pinv,0,sizeof(pinv));
        for(i=0;i<OA_DOF;++i) if(!(active&(1u<<i))) for(r=0;r<3;++r) { double b[3]={j[r][i]/o->posture_weight[i],0,0}; double sol[3]; b[0]=j[0][i]/o->posture_weight[i]; b[1]=j[1][i]/o->posture_weight[i]; b[2]=j[2][i]/o->posture_weight[i]; if(!solve3(a,b,sol)){st=OA_ESINGULAR;goto done;} pinv[i][r]=sol[r]; }
        { double posture_gain = 0.1 * norm3(e);
          for(i=0;i<OA_DOF;++i) if(!(active&(1u<<i))) { double task=0.0, post=posture_gain*(o->posture[i]-q[i]); for(r=0;r<3;++r) task+=pinv[i][r]*e[r]; nullv[i]=post; for(k=0;k<OA_DOF;++k) if(!(active&(1u<<k))) for(r=0;r<3;++r) nullv[i]-=pinv[i][r]*j[r][k]*(posture_gain*(o->posture[k]-q[k])); step[i]=task+nullv[i]; }
        }
        { double maxstep=0.0, alpha=1.0, old; for(i=0;i<OA_DOF;++i) if(fabs(step[i])>maxstep)maxstep=fabs(step[i]); if(maxstep>o->max_joint_step_rad)alpha=o->max_joint_step_rad/maxstep; for(i=0;i<OA_DOF;++i) if(!(active&(1u<<i)) && fabs(step[i])>1e-14) { double bound=step[i]>0.0 ? (m->upper[i]-o->limit_margin_rad-q[i])/step[i] : (m->lower[i]+o->limit_margin_rad-q[i])/step[i]; if(bound>=0.0 && bound<alpha) alpha=bound; }
          if(alpha < 1e-10) { for(i=0;i<OA_DOF;++i) if(!(active&(1u<<i)) && ((step[i]>0.0 && q[i]>=m->upper[i]-o->limit_margin_rad-1e-10)||(step[i]<0.0 && q[i]<=m->lower[i]+o->limit_margin_rad+1e-10))) active|=(1u<<i); if(active==0x7fu){st=OA_ESTAGNATED_AT_BOUNDS;break;} continue; }
          old=merit(e,q,o); for(;;) { double ce[3]; for(i=0;i<OA_DOF;++i)candidate[i]=q[i]+alpha*step[i]; oa_fk(m,candidate,&fk); pos(&fk.hand_tcp,p); for(r=0;r<3;++r)ce[r]=target[r]-p[r]; if(merit(ce,candidate,o)<old || alpha<1e-5) { memcpy(q,candidate,sizeof(q)); if(alpha<1e-5)st=OA_ENOCONVERGENCE; break; } alpha*=0.5; }
        }
    }
done:
    oa_fk(m,q,&fk); pos(&fk.hand_tcp,out->achieved_position_m); out->achieved_hand_tcp=fk.hand_tcp; for(r=0;r<3;++r)e[r]=target[r]-out->achieved_position_m[r]; out->position_error_m=norm3(e); out->iterations=iter; out->active_limit_mask=active; out->min_singular_value=sigma; memcpy(out->q,q,sizeof(q)); if(out->position_error_m<=o->position_tolerance_m) st=OA_OK; else if(st==OA_OK) st=OA_ENOCONVERGENCE; if(st==OA_ENOCONVERGENCE && iter==o->max_iterations) st=OA_EBUDGET; out->status=st; return st;
}
