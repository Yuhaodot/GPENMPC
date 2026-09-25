// Host numerical gateway for PX4 sensor filters.
#include "mex.h"
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <mathlib/math/filter/NotchFilter.hpp>
#include <mathlib/math/filter/AlphaFilter.hpp>
#include <matrix/matrix/math.hpp>

using matrix::Vector3f;
using matrix::Dcmf;
static void require(bool value,const char *message) {
    if (!value) { throw std::runtime_error(message); }
}
static const mxArray *field(const mxArray *s,const char *name) {
    require(mxIsStruct(s)&&mxGetNumberOfElements(s)==1,"Expected scalar struct");
    const mxArray *v=mxGetField(s,0,name);require(v!=nullptr,name);return v;
}
static const double *numbers(const mxArray *a,size_t n) {
    require(mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a)&&mxGetNumberOfElements(a)==n,
            "Expected explicit real double value of exact size");
    const double *v=mxGetPr(a);
    for (size_t k=0;k<n;++k) {
        require(std::isfinite(v[k])&&std::isfinite(static_cast<float>(v[k])),
                "Nonfinite or float-overflow numeric input");
    }
    return v;
}
static float scalar(const mxArray *s,const char *name) {
    return static_cast<float>(numbers(field(s,name),1)[0]);
}
static Vector3f vector3(const mxArray *s,const char *name) {
    const double *v=numbers(field(s,name),3);
    return Vector3f(static_cast<float>(v[0]),static_cast<float>(v[1]),static_cast<float>(v[2]));
}
static bool boolean(const mxArray *s,const char *name) {
    const mxArray *v=field(s,name);require(mxIsLogicalScalar(v),"Boolean must be scalar logical");
    return mxIsLogicalScalarTrue(v);
}
static uint64_t timestamp(const mxArray *s,const char *name) {
    const mxArray *v=field(s,name);
    require(mxIsUint64(v)&&!mxIsComplex(v)&&mxGetNumberOfElements(v)==1,"Timestamp must be scalar uint64 microseconds");
    const uint64_t t=*static_cast<const uint64_t *>(mxGetData(v));
    require(t<=UINT64_C(9007199254740991),"Timestamp outside exact diagnostic double reporting range");
    return t;
}
static std::string text(const mxArray *a) {
    require(mxIsChar(a),"Text must be MATLAB char");char *s=mxArrayToString(a);
    require(s!=nullptr,"Text conversion failed");std::string result(s);mxFree(s);return result;
}
static mxArray *vectorOutput(const Vector3f &v) {
    mxArray *a=mxCreateDoubleMatrix(3,1,mxREAL);
    for (int k=0;k<3;++k) { require(std::isfinite(v(k)),"Nonfinite filter result");mxGetPr(a)[k]=v(k); }
    return a;
}
static mxArray *uint64Output(uint64_t v) {
    mxArray *a=mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL);
    *static_cast<uint64_t *>(mxGetData(a))=v;return a;
}
static void put(mxArray *s,const char *name,mxArray *v) { mxSetField(s,0,name,v); }

struct SensorFilter {
    math::LowPassFilter2p<float> velocityLp[3];
    math::NotchFilter<float> notch0[3],notch1[3];
    AlphaFilter<float> accelerationLp[3];
    Vector3f previousFiltered{},offset{},bias{};
    Dcmf rotation;
    float fs{},gyroCutoff{},dgyroCutoff{},nf0{},bw0{},nf1{},bw1{};
    bool derivativeLpEnabled{false},haveSample{false},sequenceFault{false};
    uint64_t lastSample{0},sampleCount{0},resetCount{0};
    std::string provenance;

    explicit SensorFilter(const mxArray *cfg) {
        require(numbers(field(cfg,"dynamic_notch_enable"),1)[0]==0.0,
                "Only explicit DNF_EN=0 is supported; dynamic state cannot be invented");
        fs=scalar(cfg,"sample_rate_hz");
        require(fs>10.f&&fs<10000.f,"sample_rate_hz must satisfy source VehicleAngularVelocity selection range");
        gyroCutoff=scalar(cfg,"gyro_cutoff_hz");dgyroCutoff=scalar(cfg,"dgyro_cutoff_hz");
        nf0=scalar(cfg,"notch0_frequency_hz");bw0=scalar(cfg,"notch0_bandwidth_hz");
        nf1=scalar(cfg,"notch1_frequency_hz");bw1=scalar(cfg,"notch1_bandwidth_hz");
        offset=vector3(cfg,"offset_sensor");bias=vector3(cfg,"bias_body");
        const double *scale=numbers(field(cfg,"scale"),3);
        for (int k=0;k<3;++k) {
            require(scale[k]==1.0,"This PX4 Gyroscope::Correct has no scale term; only explicit unity scale is valid");
        }
        const mxArray *r=field(cfg,"mount_rotation");
        require(mxGetM(r)==3&&mxGetN(r)==3,"mount_rotation must be 3x3 sensor-to-body");
        const double *v=numbers(r,9);
        for (int c=0;c<3;++c) { for (int row=0;row<3;++row) { rotation(row,c)=static_cast<float>(v[row+3*c]); } }
        const Dcmf gram=rotation.transpose()*rotation;
        for (int c=0;c<3;++c) { for (int row=0;row<3;++row) {
            require(std::abs(gram(row,c)-(row==c?1.f:0.f))<=1e-5f,"mount_rotation must be orthonormal");
        } }
        const float determinant=rotation(0,0)*(rotation(1,1)*rotation(2,2)-rotation(1,2)*rotation(2,1))
            -rotation(0,1)*(rotation(1,0)*rotation(2,2)-rotation(1,2)*rotation(2,0))
            +rotation(0,2)*(rotation(1,0)*rotation(2,1)-rotation(1,1)*rotation(2,0));
        require(std::abs(determinant-1.f)<=1e-5f,"mount_rotation must be proper, determinant +1");
        provenance=text(field(cfg,"parameter_provenance"));
        require(provenance=="EXPLICIT_HOST_FIXTURE"||provenance=="CURRENT_TYPED_WITH_OBSERVED_SENSOR_RATE",
                "Explicit provenance required; no board defaults");
        resetFilters(vector3(cfg,"initial_gyro_uncalibrated"),vector3(cfg,"initial_acceleration_uncalibrated"),true);
    }

    void resetFilters(const Vector3f &gyro,const Vector3f &acceleration,bool resetClock) {
        // Use the VAV filter setters and reset methods with explicit raw-axis initial values.
        for (int a=0;a<3;++a) {
            velocityLp[a].set_cutoff_frequency(fs,gyroCutoff);velocityLp[a].reset(gyro(a));
            notch0[a].setParameters(fs,nf0,bw0);notch0[a].reset();
            notch1[a].setParameters(fs,nf1,bw1);notch1[a].reset();
            const bool enabled=dgyroCutoff>0.f&&accelerationLp[a].setCutoffFreq(fs,dgyroCutoff);
            if (enabled) { accelerationLp[a].reset(acceleration(a)); }
            else { accelerationLp[a].setAlpha(1.f); }
            derivativeLpEnabled=enabled;
        }
        previousFiltered=gyro;sequenceFault=false;++resetCount;
        if (resetClock) { lastSample=0;haveSample=false;sampleCount=0; }
    }

    mxArray *reset(const mxArray *r) {
        const Vector3f gyro=vector3(r,"gyro_uncalibrated"),alpha=vector3(r,"acceleration_uncalibrated");
        const bool resetClock=boolean(r,"reset_source_timestamp");
        require(!sequenceFault||resetClock,"Source-sequence fault requires an explicit new source lifecycle");
        resetFilters(gyro,alpha,resetClock);return status();
    }

    mxArray *step(const mxArray *s) {
        require(!sequenceFault,"Prior source-sequence fault latched; reset source lifecycle explicitly");
        const Vector3f raw=vector3(s,"raw_gyro");const uint64_t t=timestamp(s,"timestamp_sample_us");
        const bool nonprogress=haveSample&&t<=lastSample;
        const bool synthetic=lastSample==0||t<=lastSample;
        const bool prior=haveSample;const uint64_t oldSample=lastSample;
        const double observedDt=prior?(static_cast<double>(t)-static_cast<double>(oldSample))*1e-6:0.0;
        uint64_t effectivePrevious=lastSample;
        if (synthetic) {
            // Deliberately same native float expression as VAV.cpp:878.
            const float previousNative=t-1e6f/fs;
            require(std::isfinite(previousNative)&&previousNative>=0.f,
                    "First/restarted timestamp too small for native fs backfill; use a declared positive epoch offset");
            effectivePrevious=static_cast<uint64_t>(previousNative);
        }
        require(effectivePrevious<=t,"Native synthetic predecessor is later than sample");
        const float unboundedDt=(t-effectivePrevious)*1e-6f;
        const float dt=math::constrain(unboundedDt,.00002f,.02f),inverseDt=1.f/dt;
        const Vector3f previousUsed=previousFiltered;
        Vector3f filtered{},derivativeBeforeLp{},filteredDerivative{};
        for (int a=0;a<3;++a) {
            float data[1] {raw(a)};
            // Exact static part of VAV.cpp 725-781; D sees the SAME in-place
            // data AFTER NF0/NF1/gyro-LP, not raw gyro or analytic plant alpha.
            if (notch0[a].getNotchFreq()>0.f) { notch0[a].applyArray(data,1); }
            if (notch1[a].getNotchFreq()>0.f) { notch1[a].applyArray(data,1); }
            velocityLp[a].applyArray(data,1);filtered(a)=data[0];
            derivativeBeforeLp(a)=(data[0]-previousFiltered(a))*inverseDt;
            filteredDerivative(a)=accelerationLp[a].update(derivativeBeforeLp(a));
            previousFiltered(a)=data[0];
        }
        // Gyroscope.hpp Correct: R*(data - thermal_offset - offset); VAV
        // subtracts body EKF bias after that. offset_sensor is the frozen sum.
        const Vector3f rate=rotation*(filtered-offset)-bias;
        const Vector3f derivative=rotation*filteredDerivative;
        for (int a=0;a<3;++a) {
            require(std::isfinite(rate(a))&&std::isfinite(derivative(a))&&std::isfinite(derivativeBeforeLp(a)),
                    "Nonfinite filter/calibration result");
        }
        lastSample=t;haveSample=true;sequenceFault=nonprogress;++sampleCount;
        const char *reason=nonprogress?"SOURCE_NONPROGRESS_NATIVE_DT_BACKFILL_HOST_STOP_LATCH":
            (synthetic?"FIRST_SOURCE_SAMPLE_NATIVE_FS_BACKFILL":"SOURCE_PROGRESS_STATIC_FILTER_CHAIN");
        const char *names[]={"timestamp_sample_us","previous_timestamp_sample_us","effective_previous_timestamp_us",
            "had_previous_sample","raw_observed_dt_s","native_unbounded_dt_s","native_effective_dt_s","native_dt_clamped",
            "native_dt_backfilled","source_progressed","must_stop","raw_gyro_sensor","previous_filtered_gyro_sensor",
            "filtered_gyro_sensor","difference_derivative_sensor","filtered_derivative_sensor","rate_body","derivative_body",
            "sample_count","reset_count","filter_state_reason","parameter_provenance","claim"};
        mxArray *out=mxCreateStructMatrix(1,1,23,names);
        put(out,"timestamp_sample_us",uint64Output(t));put(out,"previous_timestamp_sample_us",uint64Output(oldSample));
        put(out,"effective_previous_timestamp_us",uint64Output(effectivePrevious));
        put(out,"had_previous_sample",mxCreateLogicalScalar(prior));put(out,"raw_observed_dt_s",mxCreateDoubleScalar(observedDt));
        put(out,"native_unbounded_dt_s",mxCreateDoubleScalar(unboundedDt));put(out,"native_effective_dt_s",mxCreateDoubleScalar(dt));
        put(out,"native_dt_clamped",mxCreateLogicalScalar(unboundedDt<.00002f||unboundedDt>.02f));
        put(out,"native_dt_backfilled",mxCreateLogicalScalar(synthetic));put(out,"source_progressed",mxCreateLogicalScalar(!nonprogress));
        put(out,"must_stop",mxCreateLogicalScalar(sequenceFault));put(out,"raw_gyro_sensor",vectorOutput(raw));
        put(out,"previous_filtered_gyro_sensor",vectorOutput(previousUsed));put(out,"filtered_gyro_sensor",vectorOutput(filtered));
        put(out,"difference_derivative_sensor",vectorOutput(derivativeBeforeLp));put(out,"filtered_derivative_sensor",vectorOutput(filteredDerivative));
        put(out,"rate_body",vectorOutput(rate));put(out,"derivative_body",vectorOutput(derivative));
        put(out,"sample_count",uint64Output(sampleCount));put(out,"reset_count",uint64Output(resetCount));
        put(out,"filter_state_reason",mxCreateString(reason));put(out,"parameter_provenance",mxCreateString(provenance.c_str()));
        put(out,"claim",mxCreateString("PX4_STATIC_SENSOR_FILTER_NUMERICAL_FIXTURE"));
        return out;
    }

    mxArray *status() const {
        const char *names[]={"initialized","sample_rate_hz","gyro_lowpass_enabled","notch0_enabled","notch1_enabled",
            "derivative_lowpass_enabled","dynamic_notch_enable","source_fault_latched","sample_count","reset_count",
            "previous_timestamp_sample_us","previous_filtered_gyro_sensor","parameter_provenance","source_binding_policy","claim"};
        mxArray *out=mxCreateStructMatrix(1,1,15,names);
        put(out,"initialized",mxCreateLogicalScalar(true));put(out,"sample_rate_hz",mxCreateDoubleScalar(fs));
        put(out,"gyro_lowpass_enabled",mxCreateLogicalScalar(velocityLp[0].get_cutoff_freq()>0.f));
        put(out,"notch0_enabled",mxCreateLogicalScalar(notch0[0].getNotchFreq()>0.f));
        put(out,"notch1_enabled",mxCreateLogicalScalar(notch1[0].getNotchFreq()>0.f));
        put(out,"derivative_lowpass_enabled",mxCreateLogicalScalar(derivativeLpEnabled));put(out,"dynamic_notch_enable",mxCreateDoubleScalar(0));
        put(out,"source_fault_latched",mxCreateLogicalScalar(sequenceFault));put(out,"sample_count",uint64Output(sampleCount));
        put(out,"reset_count",uint64Output(resetCount));put(out,"previous_timestamp_sample_us",uint64Output(lastSample));
        put(out,"previous_filtered_gyro_sensor",vectorOutput(previousFiltered));
        put(out,"parameter_provenance",mxCreateString(provenance.c_str()));
        put(out,"source_binding_policy",mxCreateString("CALLER_MUST_VERIFY_API_DOCUMENT_SOURCE_SHA_BEFORE_COMPILATION"));
        put(out,"claim",mxCreateString("PX4_STATIC_SENSOR_FILTER_NUMERICAL_FIXTURE"));return out;
    }
};

static std::unique_ptr<SensorFilter> filter;
static bool exitRegistered=false;
static void cleanup() { filter.reset(); }
void mexFunction(int nlhs,mxArray **plhs,int nrhs,const mxArray **prhs) {
    try {
        require(nrhs>=1,"Command required");const std::string command=text(prhs[0]);
        if (command=="clear") { require(nrhs==1&&nlhs==0,"clear takes no input/output");cleanup();return; }
        if (command=="status") { require(nrhs==1&&nlhs==1&&filter!=nullptr,"status requires initialized fixture and one output");plhs[0]=filter->status();return; }
        require(nrhs==2&&nlhs==1,"init/reset/step take one scalar struct and one output");
        if (command=="init") {
            std::unique_ptr<SensorFilter> candidate(new SensorFilter(prhs[1]));filter=std::move(candidate);
            if (!exitRegistered) { mexAtExit(cleanup);exitRegistered=true; }
            plhs[0]=filter->status();return;
        }
        require(filter!=nullptr,"Fixture not initialized");
        if (command=="step") { plhs[0]=filter->step(prhs[1]);return; }
        if (command=="reset") { plhs[0]=filter->reset(prhs[1]);return; }
        throw std::runtime_error("Unknown command");
    } catch (const std::exception &e) {
        cleanup();mexErrMsgIdAndTxt("gpenmpc:NativeSensorFilter","%s",e.what());
    }
}
