// Read the FS-i6S USB HID device through SDL2.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <mex.h>
#include <cstdint>
#include <cstring>
#include <string>

namespace {
HMODULE library=nullptr;
void *joystick=nullptr;
bool initialized=false,locked=false;
int deviceAxes=0,deviceButtons=0,deviceId=-1;
std::string deviceName;
int (*init)(std::uint32_t)=nullptr;
void (*quit)(std::uint32_t)=nullptr;
int (*hint)(const char*,const char*)=nullptr;
const char *(*error)()=nullptr;
int (*count)()=nullptr;
std::uint16_t (*vendor)(int)=nullptr;
std::uint16_t (*product)(int)=nullptr;
const char *(*name)(int)=nullptr;
void *(*open)(int)=nullptr;
void (*close)(void*)=nullptr;
void (*update)()=nullptr;
int (*attached)(void*)=nullptr;
int (*numAxes)(void*)=nullptr;
int (*numButtons)(void*)=nullptr;
int (*instance)(void*)=nullptr;
std::int16_t (*axis)(void*,int)=nullptr;
std::uint8_t (*button)(void*,int)=nullptr;
constexpr std::uint32_t subsystem=0x00000200; // SDL_INIT_JOYSTICK

void cleanup(){
    if(joystick&&close)close(joystick);
    joystick=nullptr;
    if(initialized&&quit)quit(subsystem);
    initialized=false;
    if(library)FreeLibrary(library);
    library=nullptr;
    if(locked){mexUnlock();locked=false;}
}
template<class T> void symbol(T& target,const char* key){
    auto address=GetProcAddress(library,key);
    static_assert(sizeof target==sizeof address,"function pointer size");
    std::memcpy(&target,&address,sizeof target);
    if(!target){cleanup();mexErrMsgIdAndTxt("gpenmpcRc:SdlSymbol","Missing SDL2 function %s",key);}
}
std::string sdlError(){return error?std::string(error()):std::string("SDL2 unavailable");}
void connect(const mxArray* path){
    if(library||joystick)mexErrMsgIdAndTxt("gpenmpcRc:AlreadyOpen","Close the current reader before opening another.");
    if(!mxIsChar(path)||mxGetM(path)!=1)mexErrMsgIdAndTxt("gpenmpcRc:LibraryPath","An absolute SDL2 DLL character path is required.");
    const auto n=mxGetNumberOfElements(path);const auto* chars=mxGetChars(path);
    std::wstring full;full.reserve(n);for(std::size_t k=0;k<n;++k)full.push_back(static_cast<wchar_t>(chars[k]));
    if(full.size()<4||full[1]!=L':')mexErrMsgIdAndTxt("gpenmpcRc:LibraryPath","DLL path must be absolute.");
    library=LoadLibraryExW(full.c_str(),nullptr,LOAD_WITH_ALTERED_SEARCH_PATH);
    if(!library)mexErrMsgIdAndTxt("gpenmpcRc:LibraryLoad","Cannot load installed SDL2 DLL (Windows error %lu).",GetLastError());
    symbol(init,"SDL_InitSubSystem");symbol(quit,"SDL_QuitSubSystem");symbol(hint,"SDL_SetHint");
    symbol(error,"SDL_GetError");symbol(count,"SDL_NumJoysticks");
    symbol(vendor,"SDL_JoystickGetDeviceVendor");symbol(product,"SDL_JoystickGetDeviceProduct");
    symbol(name,"SDL_JoystickNameForIndex");symbol(open,"SDL_JoystickOpen");symbol(close,"SDL_JoystickClose");
    symbol(update,"SDL_JoystickUpdate");symbol(attached,"SDL_JoystickGetAttached");
    symbol(numAxes,"SDL_JoystickNumAxes");symbol(numButtons,"SDL_JoystickNumButtons");
    symbol(instance,"SDL_JoystickInstanceID");symbol(axis,"SDL_JoystickGetAxis");symbol(button,"SDL_JoystickGetButton");
    hint("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS","1");
    if(init(subsystem)<0){auto message=sdlError();cleanup();mexErrMsgIdAndTxt("gpenmpcRc:SdlInit","%s",message.c_str());}
    initialized=true;update();int selected=-1,matches=0;
    for(int k=0;k<count();++k)if(vendor(k)==0x284e&&product(k)==0x7fff){selected=k;++matches;}
    if(matches!=1){cleanup();mexErrMsgIdAndTxt("gpenmpcRc:DeviceIdentity","Expected exactly one FS-i6S USB VID 284E PID 7FFF; found %d.",matches);}
    deviceName=name(selected)?name(selected):"";
    joystick=open(selected);
    if(!joystick){auto message=sdlError();cleanup();mexErrMsgIdAndTxt("gpenmpcRc:Open","%s",message.c_str());}
    deviceAxes=numAxes(joystick);deviceButtons=numButtons(joystick);deviceId=instance(joystick);
    if(deviceAxes<4||deviceAxes>32||deviceButtons<0||deviceButtons>64){cleanup();mexErrMsgIdAndTxt("gpenmpcRc:DeviceShape","Unexpected joystick axis/button dimensions.");}
    mexLock();locked=true;
}
mxArray* sample(){
    if(!joystick)mexErrMsgIdAndTxt("gpenmpcRc:NotOpen","Open the USB reader first.");
    update();const bool online=attached(joystick)!=0;
    LARGE_INTEGER tick,frequency;QueryPerformanceCounter(&tick);QueryPerformanceFrequency(&frequency);
    const char* fields[]={"name","vendor_id","product_id","instance_id","attached","axes_raw","buttons","host_read_qpc_s","time_basis","finish_requested"};
    auto* out=mxCreateStructMatrix(1,1,10,fields);
    mxSetField(out,0,"name",mxCreateString(deviceName.c_str()));
    mxSetField(out,0,"vendor_id",mxCreateDoubleScalar(0x284e));mxSetField(out,0,"product_id",mxCreateDoubleScalar(0x7fff));
    mxSetField(out,0,"instance_id",mxCreateDoubleScalar(deviceId));mxSetField(out,0,"attached",mxCreateLogicalScalar(online));
    auto* axes=mxCreateDoubleMatrix(1,deviceAxes,mxREAL);auto* values=mxGetPr(axes);
    for(int k=0;k<deviceAxes;++k)values[k]=online?static_cast<double>(axis(joystick,k)):mxGetNaN();
    auto* buttons=mxCreateLogicalMatrix(1,deviceButtons);auto* bs=mxGetLogicals(buttons);
    for(int k=0;k<deviceButtons;++k)bs[k]=online&&button(joystick,k)!=0;
    mxSetField(out,0,"axes_raw",axes);mxSetField(out,0,"buttons",buttons);
    mxSetField(out,0,"host_read_qpc_s",mxCreateDoubleScalar(static_cast<double>(tick.QuadPart)/frequency.QuadPart));
    mxSetField(out,0,"time_basis",mxCreateString("HOST_MONOTONIC_READ_TIME_NOT_USB_REPORT_OR_BOARD_SAMPLE_TIME"));
    // Read the manual LAND shortcut without collecting key text or sending commands.
    const bool finish=(GetAsyncKeyState(VK_CONTROL)&0x8000)&&
        (GetAsyncKeyState(VK_SHIFT)&0x8000)&&(GetAsyncKeyState('L')&0x8000);
    mxSetField(out,0,"finish_requested",mxCreateLogicalScalar(finish));
    return out;
}
}
void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
    if(nrhs<1||!mxIsChar(prhs[0])||nlhs>1)mexErrMsgIdAndTxt("gpenmpcRc:Arguments","Use open(path), read, or close with at most one output.");
    char command[16]{};mxGetString(prhs[0],command,sizeof command);
    if(!std::strcmp(command,"close")){if(nrhs!=1)mexErrMsgIdAndTxt("gpenmpcRc:Arguments","close takes no extra arguments.");cleanup();return;}
    if(!std::strcmp(command,"open")){if(nrhs!=2)mexErrMsgIdAndTxt("gpenmpcRc:Arguments","open requires the installed SDL2 DLL path.");mexAtExit(cleanup);connect(prhs[1]);}
    else if(std::strcmp(command,"read")||nrhs!=1)mexErrMsgIdAndTxt("gpenmpcRc:Arguments","Unknown joystick operation.");
    auto* out=sample();if(nlhs)plhs[0]=out;else mxDestroyArray(out);
}
