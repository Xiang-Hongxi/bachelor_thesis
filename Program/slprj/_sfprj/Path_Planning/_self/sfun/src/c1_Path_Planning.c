/* Include files */

#include <stddef.h>
#include "blas.h"
#include "Path_Planning_sfun.h"
#include "c1_Path_Planning.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Path_Planning_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c1_debug_family_names[36] = { "vlimit", "dist_thr",
  "ang_thr", "method", "step", "stop", "Kv_1", "Kw_1", "Kv_2", "Kw_2", "Psi_d",
  "finished_path_flag", "rx", "ry", "rtheta", "target_xy", "n", "xy", "i", "j",
  "nargin", "nargout", "target_list", "robot_x", "robot_y", "robot_theta",
  "pre_states", "Vf", "W", "target_distance", "ang_to_tar", "tx", "ty", "states",
  "Vf_1", "end_flag" };

static const char * c1_b_debug_family_names[4] = { "nargin", "nargout", "x", "y"
};

static const char * c1_c_debug_family_names[7] = { "nargin", "nargout", "rx",
  "ry", "tx", "ty", "dist" };

static const char * c1_d_debug_family_names[10] = { "X", "Y", "nargin",
  "nargout", "rx", "ry", "rtheta", "tx", "ty", "theta" };

static const char * c1_e_debug_family_names[14] = { "d", "vmax", "wmax",
  "nargin", "nargout", "dist", "theta", "vlimit", "ang_thr", "Kv", "Kw", "Vf_1",
  "Vf", "W" };

/* Function Declarations */
static void initialize_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance);
static void initialize_params_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct *
  chartInstance);
static void enable_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance);
static void disable_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance);
static void c1_update_debugger_state_c1_Path_Planning
  (SFc1_Path_PlanningInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_Path_Planning
  (SFc1_Path_PlanningInstanceStruct *chartInstance);
static void set_sim_state_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_st);
static void finalize_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance);
static void sf_gateway_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance);
static void c1_chartstep_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance);
static void initSimStructsc1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real_T c1_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_b_end_flag, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_c_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_b_Vf_1, const char_T *c1_identifier);
static real_T c1_d_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_e_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_states, const char_T *c1_identifier);
static real_T c1_f_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_g_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[2]);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_info_helper(const mxArray **c1_info);
static const mxArray *c1_emlrt_marshallOut(const char * c1_u);
static const mxArray *c1_b_emlrt_marshallOut(const uint32_T c1_u);
static real_T c1_check_angle(SFc1_Path_PlanningInstanceStruct *chartInstance,
  real_T c1_x);
static real_T c1_mpower(SFc1_Path_PlanningInstanceStruct *chartInstance, real_T
  c1_a);
static void c1_eml_scalar_eg(SFc1_Path_PlanningInstanceStruct *chartInstance);
static void c1_eml_error(SFc1_Path_PlanningInstanceStruct *chartInstance);
static void c1_ClosedLoop_Sequential(SFc1_Path_PlanningInstanceStruct
  *chartInstance, real_T c1_dist, real_T c1_theta, real_T c1_vlimit, real_T
  c1_ang_thr, real_T c1_Kv, real_T c1_Kw, real_T c1_b_Vf_1, real_T *c1_Vf,
  real_T *c1_W);
static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_h_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_i_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_Path_Planning, const char_T
  *c1_identifier);
static uint8_T c1_j_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_Path_PlanningInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance)
{
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_Vf_1_not_empty = false;
  chartInstance->c1_end_flag_not_empty = false;
  chartInstance->c1_is_active_c1_Path_Planning = 0U;
}

static void initialize_params_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void enable_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c1_update_debugger_state_c1_Path_Planning
  (SFc1_Path_PlanningInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c1_Path_Planning
  (SFc1_Path_PlanningInstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_hoistedGlobal;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_c_hoistedGlobal;
  real_T c1_c_u;
  const mxArray *c1_d_y = NULL;
  real_T c1_d_hoistedGlobal;
  real_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  real_T c1_e_hoistedGlobal;
  real_T c1_e_u;
  const mxArray *c1_f_y = NULL;
  real_T c1_f_hoistedGlobal;
  real_T c1_f_u;
  const mxArray *c1_g_y = NULL;
  real_T c1_g_hoistedGlobal;
  real_T c1_g_u;
  const mxArray *c1_h_y = NULL;
  real_T c1_h_hoistedGlobal;
  real_T c1_h_u;
  const mxArray *c1_i_y = NULL;
  real_T c1_i_hoistedGlobal;
  real_T c1_i_u;
  const mxArray *c1_j_y = NULL;
  uint8_T c1_j_hoistedGlobal;
  uint8_T c1_j_u;
  const mxArray *c1_k_y = NULL;
  real_T *c1_Vf;
  real_T *c1_W;
  real_T *c1_ang_to_tar;
  real_T *c1_states;
  real_T *c1_target_distance;
  real_T *c1_tx;
  real_T *c1_ty;
  c1_states = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c1_ty = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c1_tx = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c1_ang_to_tar = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c1_target_distance = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_W = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_Vf = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellmatrix(10, 1), false);
  c1_hoistedGlobal = *c1_Vf;
  c1_u = c1_hoistedGlobal;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_hoistedGlobal = *c1_W;
  c1_b_u = c1_b_hoistedGlobal;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  c1_c_hoistedGlobal = *c1_ang_to_tar;
  c1_c_u = c1_c_hoistedGlobal;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_d_hoistedGlobal = *c1_states;
  c1_d_u = c1_d_hoistedGlobal;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 3, c1_e_y);
  c1_e_hoistedGlobal = *c1_target_distance;
  c1_e_u = c1_e_hoistedGlobal;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 4, c1_f_y);
  c1_f_hoistedGlobal = *c1_tx;
  c1_f_u = c1_f_hoistedGlobal;
  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 5, c1_g_y);
  c1_g_hoistedGlobal = *c1_ty;
  c1_g_u = c1_g_hoistedGlobal;
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", &c1_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 6, c1_h_y);
  c1_h_hoistedGlobal = chartInstance->c1_Vf_1;
  c1_h_u = c1_h_hoistedGlobal;
  c1_i_y = NULL;
  if (!chartInstance->c1_Vf_1_not_empty) {
    sf_mex_assign(&c1_i_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_i_y, sf_mex_create("y", &c1_h_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 7, c1_i_y);
  c1_i_hoistedGlobal = chartInstance->c1_end_flag;
  c1_i_u = c1_i_hoistedGlobal;
  c1_j_y = NULL;
  if (!chartInstance->c1_end_flag_not_empty) {
    sf_mex_assign(&c1_j_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_j_y, sf_mex_create("y", &c1_i_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 8, c1_j_y);
  c1_j_hoistedGlobal = chartInstance->c1_is_active_c1_Path_Planning;
  c1_j_u = c1_j_hoistedGlobal;
  c1_k_y = NULL;
  sf_mex_assign(&c1_k_y, sf_mex_create("y", &c1_j_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 9, c1_k_y);
  sf_mex_assign(&c1_st, c1_y, false);
  return c1_st;
}

static void set_sim_state_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T *c1_Vf;
  real_T *c1_W;
  real_T *c1_ang_to_tar;
  real_T *c1_states;
  real_T *c1_target_distance;
  real_T *c1_tx;
  real_T *c1_ty;
  c1_states = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c1_ty = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c1_tx = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c1_ang_to_tar = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c1_target_distance = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_W = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_Vf = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c1_doneDoubleBufferReInit = true;
  c1_u = sf_mex_dup(c1_st);
  *c1_Vf = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u,
    0)), "Vf");
  *c1_W = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 1)),
    "W");
  *c1_ang_to_tar = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 2)), "ang_to_tar");
  *c1_states = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c1_u, 3)), "states");
  *c1_target_distance = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 4)), "target_distance");
  *c1_tx = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u,
    5)), "tx");
  *c1_ty = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u,
    6)), "ty");
  chartInstance->c1_Vf_1 = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 7)), "Vf_1");
  chartInstance->c1_end_flag = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 8)), "end_flag");
  chartInstance->c1_is_active_c1_Path_Planning = c1_i_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 9)),
     "is_active_c1_Path_Planning");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_Path_Planning(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance)
{
  int32_T c1_i0;
  real_T *c1_Vf;
  real_T *c1_robot_x;
  real_T *c1_robot_y;
  real_T *c1_robot_theta;
  real_T *c1_pre_states;
  real_T *c1_W;
  real_T *c1_target_distance;
  real_T *c1_ang_to_tar;
  real_T *c1_tx;
  real_T *c1_ty;
  real_T *c1_states;
  real_T (*c1_target_list)[960];
  c1_states = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c1_ty = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c1_tx = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c1_ang_to_tar = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c1_target_distance = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_W = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_pre_states = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c1_robot_theta = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_robot_y = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c1_robot_x = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c1_Vf = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_target_list = (real_T (*)[960])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i0 = 0; c1_i0 < 960; c1_i0++) {
    _SFD_DATA_RANGE_CHECK((*c1_target_list)[c1_i0], 0U);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_Path_Planning(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_Path_PlanningMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*c1_Vf, 1U);
  _SFD_DATA_RANGE_CHECK(*c1_robot_x, 2U);
  _SFD_DATA_RANGE_CHECK(*c1_robot_y, 3U);
  _SFD_DATA_RANGE_CHECK(*c1_robot_theta, 4U);
  _SFD_DATA_RANGE_CHECK(*c1_pre_states, 5U);
  _SFD_DATA_RANGE_CHECK(*c1_W, 6U);
  _SFD_DATA_RANGE_CHECK(*c1_target_distance, 7U);
  _SFD_DATA_RANGE_CHECK(*c1_ang_to_tar, 8U);
  _SFD_DATA_RANGE_CHECK(*c1_tx, 9U);
  _SFD_DATA_RANGE_CHECK(*c1_ty, 10U);
  _SFD_DATA_RANGE_CHECK(*c1_states, 11U);
}

static void c1_chartstep_c1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance)
{
  real_T c1_hoistedGlobal;
  real_T c1_b_hoistedGlobal;
  real_T c1_c_hoistedGlobal;
  real_T c1_d_hoistedGlobal;
  int32_T c1_i1;
  real_T c1_target_list[960];
  real_T c1_robot_x;
  real_T c1_robot_y;
  real_T c1_robot_theta;
  real_T c1_pre_states;
  uint32_T c1_debug_family_var_map[36];
  real_T c1_vlimit;
  real_T c1_dist_thr;
  real_T c1_ang_thr;
  real_T c1_method;
  real_T c1_step;
  real_T c1_stop;
  real_T c1_Kv_1;
  real_T c1_Kw_1;
  real_T c1_Kv_2;
  real_T c1_Kw_2;
  real_T c1_Psi_d;
  real_T c1_finished_path_flag;
  real_T c1_rx;
  real_T c1_ry;
  real_T c1_rtheta;
  real_T c1_target_xy[2];
  real_T c1_n;
  real_T c1_xy;
  real_T c1_i;
  real_T c1_j;
  real_T c1_nargin = 5.0;
  real_T c1_nargout = 7.0;
  real_T c1_Vf;
  real_T c1_W;
  real_T c1_target_distance;
  real_T c1_ang_to_tar;
  real_T c1_tx;
  real_T c1_ty;
  real_T c1_states;
  int32_T c1_b_i;
  int32_T c1_b_j;
  real_T c1_b_rx;
  real_T c1_b_ry;
  real_T c1_b_tx;
  real_T c1_b_ty;
  uint32_T c1_b_debug_family_var_map[7];
  real_T c1_b_nargin = 4.0;
  real_T c1_b_nargout = 1.0;
  real_T c1_x;
  real_T c1_b_x;
  real_T c1_c_rx;
  real_T c1_c_ry;
  real_T c1_b_rtheta;
  real_T c1_c_tx;
  real_T c1_c_ty;
  uint32_T c1_c_debug_family_var_map[10];
  real_T c1_X;
  real_T c1_Y;
  real_T c1_c_nargin = 5.0;
  real_T c1_c_nargout = 1.0;
  real_T c1_y;
  real_T c1_c_x;
  real_T c1_b_y;
  real_T c1_d_x;
  real_T c1_b_W;
  real_T c1_b_Vf;
  real_T c1_e_x;
  real_T c1_f_x;
  real_T c1_c_y;
  real_T c1_g_x;
  real_T c1_h_x;
  real_T c1_d_y;
  real_T c1_c_W;
  real_T c1_c_Vf;
  real_T *c1_b_robot_x;
  real_T *c1_b_robot_y;
  real_T *c1_b_robot_theta;
  real_T *c1_b_pre_states;
  real_T *c1_d_Vf;
  real_T *c1_d_W;
  real_T *c1_b_target_distance;
  real_T *c1_b_ang_to_tar;
  real_T *c1_d_tx;
  real_T *c1_d_ty;
  real_T *c1_b_states;
  real_T (*c1_b_target_list)[960];
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  int32_T exitg1;
  c1_b_states = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c1_d_ty = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c1_d_tx = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c1_b_ang_to_tar = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c1_b_target_distance = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_d_W = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_b_pre_states = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c1_b_robot_theta = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_b_robot_y = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c1_b_robot_x = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c1_d_Vf = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_b_target_list = (real_T (*)[960])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *c1_b_robot_x;
  c1_b_hoistedGlobal = *c1_b_robot_y;
  c1_c_hoistedGlobal = *c1_b_robot_theta;
  c1_d_hoistedGlobal = *c1_b_pre_states;
  for (c1_i1 = 0; c1_i1 < 960; c1_i1++) {
    c1_target_list[c1_i1] = (*c1_b_target_list)[c1_i1];
  }

  c1_robot_x = c1_hoistedGlobal;
  c1_robot_y = c1_b_hoistedGlobal;
  c1_robot_theta = c1_c_hoistedGlobal;
  c1_pre_states = c1_d_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 36U, 36U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_vlimit, 0U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_dist_thr, 1U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_ang_thr, 2U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_method, 3U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_step, 4U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_stop, 5U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_Kv_1, 6U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_Kw_1, 7U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Kv_2, 8U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Kw_2, 9U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_Psi_d, 10U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_finished_path_flag, 11U,
    c1_c_sf_marshallOut, c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_rx, 12U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ry, 13U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_rtheta, 14U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_target_xy, 15U, c1_e_sf_marshallOut,
    c1_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_n, 16U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_xy, 17U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i, 18U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_j, 19U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 20U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 21U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_target_list, 22U, c1_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_robot_x, 23U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_robot_y, 24U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_robot_theta, 25U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_pre_states, 26U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Vf, 27U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_W, 28U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_target_distance, 29U,
    c1_c_sf_marshallOut, c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ang_to_tar, 30U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_tx, 31U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ty, 32U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_states, 33U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_Vf_1, 34U,
    c1_b_sf_marshallOut, c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_end_flag, 35U,
    c1_sf_marshallOut, c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 3);
  c1_vlimit = 0.5;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 4);
  c1_dist_thr = 0.1;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 5);
  c1_ang_thr = 0.17453292519943295;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 6);
  c1_method = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 7);
  c1_step = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 9);
  c1_stop = 10.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 12);
  c1_Kv_1 = 2.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 13);
  c1_Kw_1 = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 15);
  c1_Kv_2 = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 16);
  c1_Kw_2 = 0.9;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 17);
  c1_Psi_d = 3.1415926535897931;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 20);
  c1_Vf = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 21);
  c1_W = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 22);
  c1_target_distance = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  c1_ang_to_tar = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 24);
  c1_finished_path_flag = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 26);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 28);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c1_Vf_1_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 29);
    chartInstance->c1_Vf_1 = 0.0;
    chartInstance->c1_Vf_1_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 31);
  if (CV_EML_IF(0, 1, 1, !chartInstance->c1_end_flag_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 32);
    chartInstance->c1_end_flag = 0.0;
    chartInstance->c1_end_flag_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 36);
  c1_states = c1_pre_states;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 38);
  c1_rx = c1_robot_x;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 39);
  c1_ry = c1_robot_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 40);
  c1_rtheta = c1_check_angle(chartInstance, c1_robot_theta);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 43);
  c1_target_xy[0] = c1_rx;
  c1_target_xy[1] = c1_ry;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 44);
  c1_n = 480.0;
  c1_xy = 2.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 45);
  CV_EML_COND(0, 1, 0, c1_n == 2.0);
  CV_EML_MCDC(0, 1, 0, false);
  CV_EML_IF(0, 1, 2, false);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 49);
  c1_i = 1.0;
  c1_b_i = 0;
  while (c1_b_i < 480) {
    c1_i = 1.0 + (real_T)c1_b_i;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 50);
    if (CV_EML_IF(0, 1, 3, c1_states == c1_i)) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 51);
      c1_j = 1.0;
      c1_b_j = 0;
      while (c1_b_j < 2) {
        c1_j = 1.0 + (real_T)c1_b_j;
        CV_EML_FOR(0, 1, 1, 1);
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 52);
        if (CV_EML_IF(0, 1, 4, c1_i < c1_n - c1_stop)) {
          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 53);
          c1_target_xy[_SFD_EML_ARRAY_BOUNDS_CHECK("target_xy", (int32_T)
            _SFD_INTEGER_CHECK("j", c1_j), 1, 2, 1, 0) - 1] = c1_target_list
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("target_list", (int32_T)
               _SFD_INTEGER_CHECK("i+stop", c1_i + c1_stop), 1, 480, 1, 0) + 480
              * (_SFD_EML_ARRAY_BOUNDS_CHECK("target_list", (int32_T)
                _SFD_INTEGER_CHECK("j", c1_j), 1, 2, 2, 0) - 1)) - 1];
        } else {
          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 55);
          c1_target_xy[_SFD_EML_ARRAY_BOUNDS_CHECK("target_xy", (int32_T)
            _SFD_INTEGER_CHECK("j", c1_j), 1, 2, 1, 0) - 1] = c1_target_list
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("target_list", (int32_T)
               _SFD_INTEGER_CHECK("i", c1_i), 1, 480, 1, 0) + 480 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("target_list", (int32_T)
                _SFD_INTEGER_CHECK("j", c1_j), 1, 2, 2, 0) - 1)) - 1];
        }

        c1_b_j++;
        _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
      }

      CV_EML_FOR(0, 1, 1, 0);
    }

    c1_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 62);
  c1_tx = c1_target_xy[0];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 63);
  c1_ty = c1_target_xy[1];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 64);
  c1_b_rx = c1_rx;
  c1_b_ry = c1_ry;
  c1_b_tx = c1_tx;
  c1_b_ty = c1_ty;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 7U, c1_c_debug_family_names,
    c1_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargin, 0U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargout, 1U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_rx, 2U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_ry, 3U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_tx, 4U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_ty, 5U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_target_distance, 6U,
    c1_c_sf_marshallOut, c1_c_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 126);
  c1_x = c1_mpower(chartInstance, c1_b_rx - c1_b_tx) + c1_mpower(chartInstance,
    c1_b_ry - c1_b_ty);
  c1_target_distance = c1_x;
  if (c1_target_distance < 0.0) {
    c1_eml_error(chartInstance);
  }

  c1_b_x = c1_target_distance;
  c1_target_distance = c1_b_x;
  c1_target_distance = muDoubleScalarSqrt(c1_target_distance);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, MAX_int8_T);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -127);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 65);
  c1_c_rx = c1_rx;
  c1_c_ry = c1_ry;
  c1_b_rtheta = c1_rtheta;
  c1_c_tx = c1_tx;
  c1_c_ty = c1_ty;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c1_d_debug_family_names,
    c1_c_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_X, 0U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Y, 1U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargin, 2U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargout, 3U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_rx, 4U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_ry, 5U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_rtheta, 6U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_tx, 7U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_ty, 8U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ang_to_tar, 9U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 132U);
  c1_X = c1_c_tx - c1_c_rx;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 133U);
  c1_Y = c1_c_ty - c1_c_ry;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 134U);
  c1_y = c1_Y;
  c1_c_x = c1_X;
  c1_eml_scalar_eg(chartInstance);
  c1_b_y = c1_y;
  c1_d_x = c1_c_x;
  c1_ang_to_tar = muDoubleScalarAtan2(c1_b_y, c1_d_x);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 135U);
  c1_ang_to_tar = c1_check_angle(chartInstance, c1_check_angle(chartInstance,
    c1_ang_to_tar) - c1_check_angle(chartInstance, c1_b_rtheta));
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 137U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -137);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 67);
  guard1 = false;
  if (CV_EML_COND(0, 1, 2, c1_states == c1_n)) {
    if (CV_EML_COND(0, 1, 3, c1_target_distance <= c1_dist_thr)) {
      CV_EML_MCDC(0, 1, 1, true);
      CV_EML_IF(0, 1, 5, true);
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 69);
      c1_Vf = 0.0;
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 70);
      c1_W = 0.0;
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 72);
      guard2 = false;
      if (CV_EML_COND(0, 1, 4, chartInstance->c1_end_flag == 1.0)) {
        if (CV_EML_COND(0, 1, 5, c1_target_distance <= c1_dist_thr)) {
          CV_EML_MCDC(0, 1, 2, true);
          CV_EML_IF(0, 1, 6, true);
          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 73);
          c1_ClosedLoop_Sequential(chartInstance, 0.0, c1_check_angle
            (chartInstance, c1_check_angle(chartInstance, 3.1415926535897931) -
             c1_check_angle(chartInstance, c1_rtheta)), 0.5, 0.17453292519943295,
            2.0, 1.0, chartInstance->c1_Vf_1, &c1_b_Vf, &c1_b_W);
          c1_Vf = c1_b_Vf;
          c1_W = c1_b_W;
          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 74);
          chartInstance->c1_Vf_1 = c1_Vf;
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }

      if (guard2 == true) {
        CV_EML_MCDC(0, 1, 2, false);
        CV_EML_IF(0, 1, 6, false);
      }
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1 == true) {
    CV_EML_MCDC(0, 1, 1, false);
    CV_EML_IF(0, 1, 5, false);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 79);
    guard3 = false;
    if (CV_EML_COND(0, 1, 6, c1_target_distance <= c1_dist_thr)) {
      if (CV_EML_COND(0, 1, 7, chartInstance->c1_end_flag == 0.0)) {
        CV_EML_MCDC(0, 1, 3, true);
        CV_EML_IF(0, 1, 7, true);
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 80);
        c1_states = c1_pre_states + c1_step;
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 81);
        if (CV_EML_IF(0, 1, 8, c1_pre_states > 0.0)) {
          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 82);
          do {
            exitg1 = 0;
            c1_e_x = c1_target_list[_SFD_EML_ARRAY_BOUNDS_CHECK("target_list",
              (int32_T)_SFD_INTEGER_CHECK("states", c1_states), 1, 480, 1, 0) -
              1] - c1_target_list[_SFD_EML_ARRAY_BOUNDS_CHECK("target_list",
              (int32_T)_SFD_INTEGER_CHECK("pre_states", c1_pre_states), 1, 480,
              1, 0) - 1];
            c1_f_x = c1_e_x;
            c1_c_y = muDoubleScalarAbs(c1_f_x);
            if (c1_c_y < 0.2) {
              c1_g_x = c1_target_list[_SFD_EML_ARRAY_BOUNDS_CHECK("target_list",
                (int32_T)_SFD_INTEGER_CHECK("states", c1_states), 1, 480, 1, 0)
                + 479] - c1_target_list[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "target_list", (int32_T)_SFD_INTEGER_CHECK("pre_states",
                c1_pre_states), 1, 480, 1, 0) + 479];
              c1_h_x = c1_g_x;
              c1_d_y = muDoubleScalarAbs(c1_h_x);
              if (c1_d_y < 0.2) {
                CV_EML_WHILE(0, 1, 0, true);
                _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 83);
                if (CV_EML_IF(0, 1, 9, c1_states <= c1_n - c1_step)) {
                  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 84);
                  c1_states += c1_step;
                } else {
                  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 86);
                  c1_states = c1_n;
                }

                _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 82);
                _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
              } else {
                exitg1 = 1;
              }
            } else {
              exitg1 = 1;
            }
          } while (exitg1 == 0);

          CV_EML_WHILE(0, 1, 0, false);
        } else {
          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 90);
          c1_states += c1_step;
        }

        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 94);
        if (CV_EML_IF(0, 1, 10, c1_states >= c1_n)) {
          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 95);
          c1_states = c1_n;
          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 96);
          chartInstance->c1_end_flag = 1.0;
        }
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }

    if (guard3 == true) {
      CV_EML_MCDC(0, 1, 3, false);
      CV_EML_IF(0, 1, 7, false);
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 100);
      CV_EML_IF(0, 1, 11, c1_method == 2.0);
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 103);
      c1_ClosedLoop_Sequential(chartInstance, c1_target_distance, c1_ang_to_tar,
        0.5, 0.17453292519943295, 2.0, 1.0, chartInstance->c1_Vf_1, &c1_c_Vf,
        &c1_c_W);
      c1_Vf = c1_c_Vf;
      c1_W = c1_c_W;
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 104);
      chartInstance->c1_Vf_1 = c1_Vf;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 110);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -110);
  _SFD_SYMBOL_SCOPE_POP();
  *c1_d_Vf = c1_Vf;
  *c1_d_W = c1_W;
  *c1_b_target_distance = c1_target_distance;
  *c1_b_ang_to_tar = c1_ang_to_tar;
  *c1_d_tx = c1_tx;
  *c1_d_ty = c1_ty;
  *c1_b_states = c1_states;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_Path_Planning(SFc1_Path_PlanningInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)c1_machineNumber;
  (void)c1_chartNumber;
  (void)c1_instanceNumber;
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_Path_PlanningInstanceStruct *chartInstance;
  chartInstance = (SFc1_Path_PlanningInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_end_flag_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_b_end_flag, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_end_flag),
    &c1_thisId);
  sf_mex_destroy(&c1_b_end_flag);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d0;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_end_flag_not_empty = false;
  } else {
    chartInstance->c1_end_flag_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d0;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_end_flag;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_Path_PlanningInstanceStruct *chartInstance;
  chartInstance = (SFc1_Path_PlanningInstanceStruct *)chartInstanceVoid;
  c1_b_end_flag = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_end_flag),
    &c1_thisId);
  sf_mex_destroy(&c1_b_end_flag);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_Path_PlanningInstanceStruct *chartInstance;
  chartInstance = (SFc1_Path_PlanningInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_Vf_1_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_c_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_b_Vf_1, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_Vf_1), &c1_thisId);
  sf_mex_destroy(&c1_b_Vf_1);
  return c1_y;
}

static real_T c1_d_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d1;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_Vf_1_not_empty = false;
  } else {
    chartInstance->c1_Vf_1_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d1, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d1;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_Vf_1;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_Path_PlanningInstanceStruct *chartInstance;
  chartInstance = (SFc1_Path_PlanningInstanceStruct *)chartInstanceVoid;
  c1_b_Vf_1 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_Vf_1), &c1_thisId);
  sf_mex_destroy(&c1_b_Vf_1);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_Path_PlanningInstanceStruct *chartInstance;
  chartInstance = (SFc1_Path_PlanningInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_e_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_states, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_states), &c1_thisId);
  sf_mex_destroy(&c1_states);
  return c1_y;
}

static real_T c1_f_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d2;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d2, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d2;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_states;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_Path_PlanningInstanceStruct *chartInstance;
  chartInstance = (SFc1_Path_PlanningInstanceStruct *)chartInstanceVoid;
  c1_states = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_states), &c1_thisId);
  sf_mex_destroy(&c1_states);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i2;
  int32_T c1_i3;
  int32_T c1_i4;
  real_T c1_b_inData[960];
  int32_T c1_i5;
  int32_T c1_i6;
  int32_T c1_i7;
  real_T c1_u[960];
  const mxArray *c1_y = NULL;
  SFc1_Path_PlanningInstanceStruct *chartInstance;
  chartInstance = (SFc1_Path_PlanningInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i2 = 0;
  for (c1_i3 = 0; c1_i3 < 2; c1_i3++) {
    for (c1_i4 = 0; c1_i4 < 480; c1_i4++) {
      c1_b_inData[c1_i4 + c1_i2] = (*(real_T (*)[960])c1_inData)[c1_i4 + c1_i2];
    }

    c1_i2 += 480;
  }

  c1_i5 = 0;
  for (c1_i6 = 0; c1_i6 < 2; c1_i6++) {
    for (c1_i7 = 0; c1_i7 < 480; c1_i7++) {
      c1_u[c1_i7 + c1_i5] = c1_b_inData[c1_i7 + c1_i5];
    }

    c1_i5 += 480;
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 480, 2), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i8;
  real_T c1_b_inData[2];
  int32_T c1_i9;
  real_T c1_u[2];
  const mxArray *c1_y = NULL;
  SFc1_Path_PlanningInstanceStruct *chartInstance;
  chartInstance = (SFc1_Path_PlanningInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i8 = 0; c1_i8 < 2; c1_i8++) {
    c1_b_inData[c1_i8] = (*(real_T (*)[2])c1_inData)[c1_i8];
  }

  for (c1_i9 = 0; c1_i9 < 2; c1_i9++) {
    c1_u[c1_i9] = c1_b_inData[c1_i9];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_g_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[2])
{
  real_T c1_dv0[2];
  int32_T c1_i10;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv0, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c1_i10 = 0; c1_i10 < 2; c1_i10++) {
    c1_y[c1_i10] = c1_dv0[c1_i10];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_target_xy;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[2];
  int32_T c1_i11;
  SFc1_Path_PlanningInstanceStruct *chartInstance;
  chartInstance = (SFc1_Path_PlanningInstanceStruct *)chartInstanceVoid;
  c1_target_xy = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_target_xy), &c1_thisId,
                        c1_y);
  sf_mex_destroy(&c1_target_xy);
  for (c1_i11 = 0; c1_i11 < 2; c1_i11++) {
    (*(real_T (*)[2])c1_outData)[c1_i11] = c1_y[c1_i11];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_Path_Planning_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_createstruct("structure", 2, 33, 1),
                false);
  c1_info_helper(&c1_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_info_helper(const mxArray **c1_info)
{
  const mxArray *c1_rhs0 = NULL;
  const mxArray *c1_lhs0 = NULL;
  const mxArray *c1_rhs1 = NULL;
  const mxArray *c1_lhs1 = NULL;
  const mxArray *c1_rhs2 = NULL;
  const mxArray *c1_lhs2 = NULL;
  const mxArray *c1_rhs3 = NULL;
  const mxArray *c1_lhs3 = NULL;
  const mxArray *c1_rhs4 = NULL;
  const mxArray *c1_lhs4 = NULL;
  const mxArray *c1_rhs5 = NULL;
  const mxArray *c1_lhs5 = NULL;
  const mxArray *c1_rhs6 = NULL;
  const mxArray *c1_lhs6 = NULL;
  const mxArray *c1_rhs7 = NULL;
  const mxArray *c1_lhs7 = NULL;
  const mxArray *c1_rhs8 = NULL;
  const mxArray *c1_lhs8 = NULL;
  const mxArray *c1_rhs9 = NULL;
  const mxArray *c1_lhs9 = NULL;
  const mxArray *c1_rhs10 = NULL;
  const mxArray *c1_lhs10 = NULL;
  const mxArray *c1_rhs11 = NULL;
  const mxArray *c1_lhs11 = NULL;
  const mxArray *c1_rhs12 = NULL;
  const mxArray *c1_lhs12 = NULL;
  const mxArray *c1_rhs13 = NULL;
  const mxArray *c1_lhs13 = NULL;
  const mxArray *c1_rhs14 = NULL;
  const mxArray *c1_lhs14 = NULL;
  const mxArray *c1_rhs15 = NULL;
  const mxArray *c1_lhs15 = NULL;
  const mxArray *c1_rhs16 = NULL;
  const mxArray *c1_lhs16 = NULL;
  const mxArray *c1_rhs17 = NULL;
  const mxArray *c1_lhs17 = NULL;
  const mxArray *c1_rhs18 = NULL;
  const mxArray *c1_lhs18 = NULL;
  const mxArray *c1_rhs19 = NULL;
  const mxArray *c1_lhs19 = NULL;
  const mxArray *c1_rhs20 = NULL;
  const mxArray *c1_lhs20 = NULL;
  const mxArray *c1_rhs21 = NULL;
  const mxArray *c1_lhs21 = NULL;
  const mxArray *c1_rhs22 = NULL;
  const mxArray *c1_lhs22 = NULL;
  const mxArray *c1_rhs23 = NULL;
  const mxArray *c1_lhs23 = NULL;
  const mxArray *c1_rhs24 = NULL;
  const mxArray *c1_lhs24 = NULL;
  const mxArray *c1_rhs25 = NULL;
  const mxArray *c1_lhs25 = NULL;
  const mxArray *c1_rhs26 = NULL;
  const mxArray *c1_lhs26 = NULL;
  const mxArray *c1_rhs27 = NULL;
  const mxArray *c1_lhs27 = NULL;
  const mxArray *c1_rhs28 = NULL;
  const mxArray *c1_lhs28 = NULL;
  const mxArray *c1_rhs29 = NULL;
  const mxArray *c1_lhs29 = NULL;
  const mxArray *c1_rhs30 = NULL;
  const mxArray *c1_lhs30 = NULL;
  const mxArray *c1_rhs31 = NULL;
  const mxArray *c1_lhs31 = NULL;
  const mxArray *c1_rhs32 = NULL;
  const mxArray *c1_lhs32 = NULL;
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("mrdivide"), "name", "name", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c1_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c1_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("rdivide"), "name", "name", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c1_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c1_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286825996U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c1_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_div"), "name", "name", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c1_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c1_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("mpower"), "name", "name", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363717478U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c1_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c1_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("ismatrix"), "name", "name", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1331308458U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c1_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("power"), "name", "name", 10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c1_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c1_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c1_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c1_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c1_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c1_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("floor"), "name", "name", 16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363717454U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c1_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c1_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286825926U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c1_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 19);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 19);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c1_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 20);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("sqrt"), "name", "name", 20);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 20);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c1_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 21);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_error"), "name", "name",
                  21);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 21);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1343837558U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c1_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 22);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286825938U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c1_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 23);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("atan2"), "name", "name", 23);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "resolved",
                  "resolved", 23);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c1_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 24);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 24);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 24);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c1_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 25);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 25);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c1_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 26);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_atan2"), "name",
                  "name", 26);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286825920U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c1_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 27);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("abs"), "name", "name", 27);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 27);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363717452U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c1_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 28);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 28);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c1_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 29);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 29);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286825912U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c1_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 30);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("sign"), "name", "name", 30);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sign.m"), "resolved",
                  "resolved", 30);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363717456U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c1_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sign.m"), "context",
                  "context", 31);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 31);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c1_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sign.m"), "context",
                  "context", 32);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_sign"), "name",
                  "name", 32);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sign.m"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1356545094U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c1_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs32), "lhs", "lhs",
                  32);
  sf_mex_destroy(&c1_rhs0);
  sf_mex_destroy(&c1_lhs0);
  sf_mex_destroy(&c1_rhs1);
  sf_mex_destroy(&c1_lhs1);
  sf_mex_destroy(&c1_rhs2);
  sf_mex_destroy(&c1_lhs2);
  sf_mex_destroy(&c1_rhs3);
  sf_mex_destroy(&c1_lhs3);
  sf_mex_destroy(&c1_rhs4);
  sf_mex_destroy(&c1_lhs4);
  sf_mex_destroy(&c1_rhs5);
  sf_mex_destroy(&c1_lhs5);
  sf_mex_destroy(&c1_rhs6);
  sf_mex_destroy(&c1_lhs6);
  sf_mex_destroy(&c1_rhs7);
  sf_mex_destroy(&c1_lhs7);
  sf_mex_destroy(&c1_rhs8);
  sf_mex_destroy(&c1_lhs8);
  sf_mex_destroy(&c1_rhs9);
  sf_mex_destroy(&c1_lhs9);
  sf_mex_destroy(&c1_rhs10);
  sf_mex_destroy(&c1_lhs10);
  sf_mex_destroy(&c1_rhs11);
  sf_mex_destroy(&c1_lhs11);
  sf_mex_destroy(&c1_rhs12);
  sf_mex_destroy(&c1_lhs12);
  sf_mex_destroy(&c1_rhs13);
  sf_mex_destroy(&c1_lhs13);
  sf_mex_destroy(&c1_rhs14);
  sf_mex_destroy(&c1_lhs14);
  sf_mex_destroy(&c1_rhs15);
  sf_mex_destroy(&c1_lhs15);
  sf_mex_destroy(&c1_rhs16);
  sf_mex_destroy(&c1_lhs16);
  sf_mex_destroy(&c1_rhs17);
  sf_mex_destroy(&c1_lhs17);
  sf_mex_destroy(&c1_rhs18);
  sf_mex_destroy(&c1_lhs18);
  sf_mex_destroy(&c1_rhs19);
  sf_mex_destroy(&c1_lhs19);
  sf_mex_destroy(&c1_rhs20);
  sf_mex_destroy(&c1_lhs20);
  sf_mex_destroy(&c1_rhs21);
  sf_mex_destroy(&c1_lhs21);
  sf_mex_destroy(&c1_rhs22);
  sf_mex_destroy(&c1_lhs22);
  sf_mex_destroy(&c1_rhs23);
  sf_mex_destroy(&c1_lhs23);
  sf_mex_destroy(&c1_rhs24);
  sf_mex_destroy(&c1_lhs24);
  sf_mex_destroy(&c1_rhs25);
  sf_mex_destroy(&c1_lhs25);
  sf_mex_destroy(&c1_rhs26);
  sf_mex_destroy(&c1_lhs26);
  sf_mex_destroy(&c1_rhs27);
  sf_mex_destroy(&c1_lhs27);
  sf_mex_destroy(&c1_rhs28);
  sf_mex_destroy(&c1_lhs28);
  sf_mex_destroy(&c1_rhs29);
  sf_mex_destroy(&c1_lhs29);
  sf_mex_destroy(&c1_rhs30);
  sf_mex_destroy(&c1_lhs30);
  sf_mex_destroy(&c1_rhs31);
  sf_mex_destroy(&c1_lhs31);
  sf_mex_destroy(&c1_rhs32);
  sf_mex_destroy(&c1_lhs32);
}

static const mxArray *c1_emlrt_marshallOut(const char * c1_u)
{
  const mxArray *c1_y = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c1_u)), false);
  return c1_y;
}

static const mxArray *c1_b_emlrt_marshallOut(const uint32_T c1_u)
{
  const mxArray *c1_y = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 7, 0U, 0U, 0U, 0), false);
  return c1_y;
}

static real_T c1_check_angle(SFc1_Path_PlanningInstanceStruct *chartInstance,
  real_T c1_x)
{
  real_T c1_y;
  uint32_T c1_debug_family_var_map[4];
  real_T c1_nargin = 1.0;
  real_T c1_nargout = 1.0;
  int32_T exitg1;
  boolean_T guard1 = false;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c1_b_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_x, 2U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_y, 3U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 114);
  c1_y = c1_x;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 115);
  do {
    exitg1 = 0;
    guard1 = false;
    if (c1_x > 3.1415926535897931) {
      guard1 = true;
    } else if (c1_x < -3.1415926535897931) {
      guard1 = true;
    } else {
      exitg1 = 1;
    }

    if (guard1 == true) {
      CV_EML_WHILE(0, 1, 1, true);
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 116);
      if (CV_EML_IF(0, 1, 12, c1_x > 3.1415926535897931)) {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 117);
        c1_y = c1_x - 6.2831853071795862;
      } else {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 119);
        c1_y = c1_x + 6.2831853071795862;
      }

      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 115);
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }
  } while (exitg1 == 0);

  CV_EML_WHILE(0, 1, 1, false);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 122);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -122);
  _SFD_SYMBOL_SCOPE_POP();
  return c1_y;
}

static real_T c1_mpower(SFc1_Path_PlanningInstanceStruct *chartInstance, real_T
  c1_a)
{
  real_T c1_b_a;
  real_T c1_c_a;
  real_T c1_ak;
  real_T c1_d_a;
  c1_b_a = c1_a;
  c1_c_a = c1_b_a;
  c1_eml_scalar_eg(chartInstance);
  c1_ak = c1_c_a;
  c1_d_a = c1_ak;
  c1_eml_scalar_eg(chartInstance);
  return c1_d_a * c1_d_a;
}

static void c1_eml_scalar_eg(SFc1_Path_PlanningInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c1_eml_error(SFc1_Path_PlanningInstanceStruct *chartInstance)
{
  int32_T c1_i12;
  static char_T c1_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c1_u[30];
  const mxArray *c1_y = NULL;
  int32_T c1_i13;
  static char_T c1_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c1_b_u[4];
  const mxArray *c1_b_y = NULL;
  (void)chartInstance;
  for (c1_i12 = 0; c1_i12 < 30; c1_i12++) {
    c1_u[c1_i12] = c1_cv0[c1_i12];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c1_i13 = 0; c1_i13 < 4; c1_i13++) {
    c1_b_u[c1_i13] = c1_cv1[c1_i13];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c1_y, 14, c1_b_y));
}

static void c1_ClosedLoop_Sequential(SFc1_Path_PlanningInstanceStruct
  *chartInstance, real_T c1_dist, real_T c1_theta, real_T c1_vlimit, real_T
  c1_ang_thr, real_T c1_Kv, real_T c1_Kw, real_T c1_b_Vf_1, real_T *c1_Vf,
  real_T *c1_W)
{
  uint32_T c1_debug_family_var_map[14];
  real_T c1_d;
  real_T c1_vmax;
  real_T c1_wmax;
  real_T c1_nargin = 8.0;
  real_T c1_nargout = 2.0;
  real_T c1_x;
  real_T c1_b_x;
  real_T c1_y;
  real_T c1_c_x;
  real_T c1_d_x;
  real_T c1_b_y;
  real_T c1_e_x;
  real_T c1_f_x;
  real_T c1_c_y;
  real_T c1_g_x;
  real_T c1_h_x;
  real_T c1_i_x;
  real_T c1_j_x;
  real_T c1_d_y;
  real_T c1_k_x;
  real_T c1_l_x;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 14U, 14U, c1_e_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_d, 0U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_vmax, 1U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_wmax, 2U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 3U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 4U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_dist, 5U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_theta, 6U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_vlimit, 7U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ang_thr, 8U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Kv, 9U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Kw, 10U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_Vf_1, 11U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_Vf, 12U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_W, 13U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  CV_EML_FCN(0, 5);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 162U);
  c1_d = 0.23;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 163U);
  c1_vmax = c1_vlimit;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 164U);
  c1_wmax = 2.0 * c1_vmax;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 166U);
  c1_x = c1_theta;
  c1_b_x = c1_x;
  c1_y = muDoubleScalarAbs(c1_b_x);
  guard1 = false;
  guard2 = false;
  if (CV_EML_COND(0, 1, 8, c1_y > c1_ang_thr)) {
    if (CV_EML_COND(0, 1, 9, c1_b_Vf_1 < 0.01)) {
      guard1 = true;
    } else {
      guard2 = true;
    }
  } else {
    guard2 = true;
  }

  if (guard2 == true) {
    c1_c_x = c1_theta;
    c1_d_x = c1_c_x;
    c1_b_y = muDoubleScalarAbs(c1_d_x);
    if (CV_EML_COND(0, 1, 10, c1_b_y > 5.0 * c1_ang_thr)) {
      guard1 = true;
    } else {
      CV_EML_MCDC(0, 1, 4, false);
      CV_EML_IF(0, 1, 15, false);
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 173U);
      *c1_Vf = c1_Kv * c1_dist;
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 175U);
      c1_i_x = *c1_Vf;
      c1_j_x = c1_i_x;
      c1_d_y = muDoubleScalarAbs(c1_j_x);
      if (CV_EML_IF(0, 1, 17, c1_d_y > c1_vlimit)) {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 176U);
        c1_k_x = *c1_Vf;
        c1_l_x = c1_k_x;
        c1_l_x = muDoubleScalarSign(c1_l_x);
        *c1_Vf = c1_l_x * c1_vlimit;
      }

      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 179U);
      *c1_W = 0.0;
    }
  }

  if (guard1 == true) {
    CV_EML_MCDC(0, 1, 4, true);
    CV_EML_IF(0, 1, 15, true);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 167U);
    *c1_W = c1_Kw * c1_theta;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 168U);
    *c1_Vf = 0.0;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 169U);
    c1_e_x = *c1_W;
    c1_f_x = c1_e_x;
    c1_c_y = muDoubleScalarAbs(c1_f_x);
    if (CV_EML_IF(0, 1, 16, c1_c_y > c1_wmax)) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 170U);
      c1_g_x = *c1_W;
      c1_h_x = c1_g_x;
      c1_h_x = muDoubleScalarSign(c1_h_x);
      *c1_W = c1_h_x * c1_wmax;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 182U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -182);
  _SFD_SYMBOL_SCOPE_POP();
}

static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_Path_PlanningInstanceStruct *chartInstance;
  chartInstance = (SFc1_Path_PlanningInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_h_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i14;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i14, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i14;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_Path_PlanningInstanceStruct *chartInstance;
  chartInstance = (SFc1_Path_PlanningInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_i_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_Path_Planning, const char_T
  *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_Path_Planning), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_Path_Planning);
  return c1_y;
}

static uint8_T c1_j_emlrt_marshallIn(SFc1_Path_PlanningInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void init_dsm_address_info(SFc1_Path_PlanningInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c1_Path_Planning_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4156212837U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1709261626U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1668358876U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4292615454U);
}

mxArray *sf_c1_Path_Planning_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("ZNTJ3kiSCpEWDMmIrGCHLD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(480);
      pr[1] = (double)(2);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,7,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_Path_Planning_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_Path_Planning_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c1_Path_Planning(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[5],T\"Vf\",},{M[1],M[8],T\"W\",},{M[1],M[10],T\"ang_to_tar\",},{M[1],M[12],T\"states\",},{M[1],M[9],T\"target_distance\",},{M[1],M[11],T\"tx\",},{M[1],M[15],T\"ty\",},{M[4],M[0],T\"Vf_1\",S'l','i','p'{{M1x2[541 545],M[0],}}},{M[4],M[0],T\"end_flag\",S'l','i','p'{{M1x2[546 554],M[0],}}},{M[8],M[0],T\"is_active_c1_Path_Planning\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 10, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_Path_Planning_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_Path_PlanningInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc1_Path_PlanningInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _Path_PlanningMachineNumber_,
           1,
           1,
           1,
           0,
           12,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_Path_PlanningMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_Path_PlanningMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _Path_PlanningMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"target_list");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Vf");
          _SFD_SET_DATA_PROPS(2,1,1,0,"robot_x");
          _SFD_SET_DATA_PROPS(3,1,1,0,"robot_y");
          _SFD_SET_DATA_PROPS(4,1,1,0,"robot_theta");
          _SFD_SET_DATA_PROPS(5,1,1,0,"pre_states");
          _SFD_SET_DATA_PROPS(6,2,0,1,"W");
          _SFD_SET_DATA_PROPS(7,2,0,1,"target_distance");
          _SFD_SET_DATA_PROPS(8,2,0,1,"ang_to_tar");
          _SFD_SET_DATA_PROPS(9,2,0,1,"tx");
          _SFD_SET_DATA_PROPS(10,2,0,1,"ty");
          _SFD_SET_DATA_PROPS(11,2,0,1,"states");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,6,18,0,0,0,2,2,11,5);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2901);
        _SFD_CV_INIT_EML_FCN(0,1,"check_angle",2901,-1,3074);
        _SFD_CV_INIT_EML_FCN(0,2,"find_dist",3074,-1,3188);
        _SFD_CV_INIT_EML_FCN(0,3,"find_theta",3188,-1,3411);
        _SFD_CV_INIT_EML_FCN(0,4,"ClosedLoop_Both",3411,-1,3726);
        _SFD_CV_INIT_EML_FCN(0,5,"ClosedLoop_Sequential",3726,-1,4183);
        _SFD_CV_INIT_EML_IF(0,1,0,559,575,-1,593);
        _SFD_CV_INIT_EML_IF(0,1,1,595,615,-1,638);
        _SFD_CV_INIT_EML_IF(0,1,2,861,877,942,1278);
        _SFD_CV_INIT_EML_IF(0,1,3,988,1005,-1,1266);
        _SFD_CV_INIT_EML_IF(0,1,4,1064,1077,1160,1238);
        _SFD_CV_INIT_EML_IF(0,1,5,1412,1466,1801,2854);
        _SFD_CV_INIT_EML_IF(0,1,6,1530,1581,-1,1796);
        _SFD_CV_INIT_EML_IF(0,1,7,1837,1886,2543,2845);
        _SFD_CV_INIT_EML_IF(0,1,8,1946,1967,2350,2408);
        _SFD_CV_INIT_EML_IF(0,1,9,2167,2191,2263,2322);
        _SFD_CV_INIT_EML_IF(0,1,10,2448,2465,-1,2537);
        _SFD_CV_INIT_EML_IF(0,1,11,2556,2570,2672,2830);
        _SFD_CV_INIT_EML_IF(0,1,12,2969,2978,3005,3039);
        _SFD_CV_INIT_EML_IF(0,1,13,3582,3598,-1,3623);
        _SFD_CV_INIT_EML_IF(0,1,14,3624,3642,-1,3698);
        _SFD_CV_INIT_EML_IF(0,1,15,3865,3938,4038,4175);
        _SFD_CV_INIT_EML_IF(0,1,16,3982,3997,-1,4037);
        _SFD_CV_INIT_EML_IF(0,1,17,4079,4096,-1,4140);
        _SFD_CV_INIT_EML_FOR(0,1,0,951,961,1274);
        _SFD_CV_INIT_EML_FOR(0,1,1,1037,1048,1254);
        _SFD_CV_INIT_EML_WHILE(0,1,0,1989,2150,2339);
        _SFD_CV_INIT_EML_WHILE(0,1,1,2938,2964,3043);

        {
          static int condStart[] = { 864, 872 };

          static int condEnd[] = { 868, 877 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,864,877,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1417, 1437 };

          static int condEnd[] = { 1431, 1464 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,1,1416,1465,2,2,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1534, 1553 };

          static int condEnd[] = { 1547, 1580 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,2,1533,1581,2,4,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1841, 1874 };

          static int condEnd[] = { 1868, 1885 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,3,1840,1886,2,6,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 3871, 3899, 3915 };

          static int condEnd[] = { 3891, 3908, 3937 };

          static int pfixExpr[] = { 0, 1, -3, 2, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,4,3868,3938,3,8,&(condStart[0]),&(condEnd[0]),
                                5,&(pfixExpr[0]));
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 480;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)c1_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)c1_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)c1_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)c1_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)c1_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)c1_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)c1_c_sf_marshallIn);

        {
          real_T *c1_Vf;
          real_T *c1_robot_x;
          real_T *c1_robot_y;
          real_T *c1_robot_theta;
          real_T *c1_pre_states;
          real_T *c1_W;
          real_T *c1_target_distance;
          real_T *c1_ang_to_tar;
          real_T *c1_tx;
          real_T *c1_ty;
          real_T *c1_states;
          real_T (*c1_target_list)[960];
          c1_states = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
          c1_ty = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
          c1_tx = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
          c1_ang_to_tar = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
          c1_target_distance = (real_T *)ssGetOutputPortSignal(chartInstance->S,
            3);
          c1_W = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c1_pre_states = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c1_robot_theta = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c1_robot_y = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c1_robot_x = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c1_Vf = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c1_target_list = (real_T (*)[960])ssGetInputPortSignal
            (chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c1_target_list);
          _SFD_SET_DATA_VALUE_PTR(1U, c1_Vf);
          _SFD_SET_DATA_VALUE_PTR(2U, c1_robot_x);
          _SFD_SET_DATA_VALUE_PTR(3U, c1_robot_y);
          _SFD_SET_DATA_VALUE_PTR(4U, c1_robot_theta);
          _SFD_SET_DATA_VALUE_PTR(5U, c1_pre_states);
          _SFD_SET_DATA_VALUE_PTR(6U, c1_W);
          _SFD_SET_DATA_VALUE_PTR(7U, c1_target_distance);
          _SFD_SET_DATA_VALUE_PTR(8U, c1_ang_to_tar);
          _SFD_SET_DATA_VALUE_PTR(9U, c1_tx);
          _SFD_SET_DATA_VALUE_PTR(10U, c1_ty);
          _SFD_SET_DATA_VALUE_PTR(11U, c1_states);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _Path_PlanningMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "AwxDslZcbIrTqGTtrVU4IG";
}

static void sf_opaque_initialize_c1_Path_Planning(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_Path_PlanningInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_Path_Planning((SFc1_Path_PlanningInstanceStruct*)
    chartInstanceVar);
  initialize_c1_Path_Planning((SFc1_Path_PlanningInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c1_Path_Planning(void *chartInstanceVar)
{
  enable_c1_Path_Planning((SFc1_Path_PlanningInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_Path_Planning(void *chartInstanceVar)
{
  disable_c1_Path_Planning((SFc1_Path_PlanningInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_Path_Planning(void *chartInstanceVar)
{
  sf_gateway_c1_Path_Planning((SFc1_Path_PlanningInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c1_Path_Planning(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_Path_Planning
    ((SFc1_Path_PlanningInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_Path_Planning();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c1_Path_Planning(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c1_Path_Planning();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_Path_Planning((SFc1_Path_PlanningInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c1_Path_Planning(SimStruct* S)
{
  return sf_internal_get_sim_state_c1_Path_Planning(S);
}

static void sf_opaque_set_sim_state_c1_Path_Planning(SimStruct* S, const mxArray
  *st)
{
  sf_internal_set_sim_state_c1_Path_Planning(S, st);
}

static void sf_opaque_terminate_c1_Path_Planning(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_Path_PlanningInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_Path_Planning_optimization_info();
    }

    finalize_c1_Path_Planning((SFc1_Path_PlanningInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_Path_Planning((SFc1_Path_PlanningInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_Path_Planning(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c1_Path_Planning((SFc1_Path_PlanningInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_Path_Planning(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Path_Planning_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,7);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=7; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(831093925U));
  ssSetChecksum1(S,(1033995661U));
  ssSetChecksum2(S,(11167426U));
  ssSetChecksum3(S,(3895248415U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_Path_Planning(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_Path_Planning(SimStruct *S)
{
  SFc1_Path_PlanningInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc1_Path_PlanningInstanceStruct *)utMalloc(sizeof
    (SFc1_Path_PlanningInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_Path_PlanningInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c1_Path_Planning;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_Path_Planning;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c1_Path_Planning;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_Path_Planning;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_Path_Planning;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_Path_Planning;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_Path_Planning;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_Path_Planning;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_Path_Planning;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_Path_Planning;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_Path_Planning;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c1_Path_Planning_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_Path_Planning(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_Path_Planning(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_Path_Planning(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_Path_Planning_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
