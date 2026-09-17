#include "esc/vesc_board.hpp"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace esc {
namespace {
constexpr std::uint8_t COMM_FW_VERSION=0, COMM_SET_RPM=8, COMM_ALIVE=30;
constexpr std::uint8_t COMM_FORWARD_CAN=34, COMM_CUSTOM_APP_DATA=36, COMM_GET_VALUES_SELECTIVE=50;
constexpr std::uint8_t RIGHT_ID=2, HB_MAGIC0=0x48, HB_MAGIC1=0x42, HB_VERSION=2;
constexpr std::uint8_t HB_SET_STEERING_DEG=9, HB_GET_STEERING_CAL=10, HB_GET_PLATFORM_INFO=25;
constexpr std::uint32_t VALUE_MASK=(1u<<0)|(1u<<2)|(1u<<3)|(1u<<4)|(1u<<5)|(1u<<6)|(1u<<7)|(1u<<8)|(1u<<15)|(1u<<16)|(1u<<17)|(1u<<19)|(1u<<20);

std::uint16_t crc16(const std::uint8_t *d, std::size_t n){
  std::uint16_t crc=0; for(std::size_t i=0;i<n;++i){crc^=static_cast<std::uint16_t>(d[i])<<8;for(int b=0;b<8;++b)crc=(crc&0x8000u)?static_cast<std::uint16_t>((crc<<1)^0x1021u):static_cast<std::uint16_t>(crc<<1);}return crc;
}
void be16(std::vector<std::uint8_t>&v,std::uint16_t x){v.push_back(x>>8);v.push_back(x&0xff);}
void be32(std::vector<std::uint8_t>&v,std::uint32_t x){v.push_back(x>>24);v.push_back((x>>16)&0xff);v.push_back((x>>8)&0xff);v.push_back(x&0xff);}
std::uint16_t u16(const std::vector<std::uint8_t>&v,std::size_t p){return static_cast<std::uint16_t>((v[p]<<8)|v[p+1]);}
std::uint32_t u32(const std::vector<std::uint8_t>&v,std::size_t p){return (static_cast<std::uint32_t>(v[p])<<24)|(static_cast<std::uint32_t>(v[p+1])<<16)|(static_cast<std::uint32_t>(v[p+2])<<8)|v[p+3];}
std::int32_t i32(const std::vector<std::uint8_t>&v,std::size_t p){return static_cast<std::int32_t>(u32(v,p));}
}

const char *vesc_role_name(VescRole r){switch(r){case VescRole::LEGACY_MIXED:return "LEGACY_MIXED";case VescRole::DRIVE_DUAL_HALL:return "DRIVE_DUAL_HALL";case VescRole::STEER_LEFT_ENCODER:return "STEER_LEFT_ENCODER";default:return "UNKNOWN";}}

VescBoard::VescBoard(std::string port,int baud):port_(std::move(port)),baud_(baud){rx_.reserve(1024);} VescBoard::~VescBoard(){close_port();}
void VescBoard::close_port(){if(fd_>=0)::close(fd_);fd_=-1;rx_.clear();role_=VescRole::UNKNOWN;role_caps_=0;steering_cal_=SteeringCal{};values_[0]=VescValues{};values_[1]=VescValues{};}

bool VescBoard::open_port(){const auto now=std::chrono::steady_clock::now();if(port_.empty()||now-last_open_try_<std::chrono::seconds(1))return false;last_open_try_=now;int f=::open(port_.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(f<0)return false;if(ioctl(f,TIOCEXCL)!=0){::close(f);return false;}termios t{};if(tcgetattr(f,&t)!=0){::close(f);return false;}cfmakeraw(&t);
#ifdef B921600
  speed_t sp=baud_==921600?B921600:B115200;
#else
  speed_t sp=B115200;
#endif
  cfsetispeed(&t,sp);cfsetospeed(&t,sp);t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;if(tcsetattr(f,TCSANOW,&t)!=0){::close(f);return false;}tcflush(f,TCIOFLUSH);fd_=f;++connection_generation_;opened_at_=last_rx_=now;rx_.clear();role_=VescRole::UNKNOWN;role_caps_=0;request_platform_info();return true;}

bool VescBoard::send_payload(const std::vector<std::uint8_t>&p){if(fd_<0||p.empty()||p.size()>65535)return false;std::vector<std::uint8_t> f; if(p.size()<=255){f.push_back(2);f.push_back(static_cast<std::uint8_t>(p.size()));}else{f.push_back(3);be16(f,static_cast<std::uint16_t>(p.size()));}f.insert(f.end(),p.begin(),p.end());const auto c=crc16(p.data(),p.size());be16(f,c);f.push_back(3);std::size_t off=0;while(off<f.size()){ssize_t n=::write(fd_,f.data()+off,f.size()-off);if(n>0){off+=static_cast<std::size_t>(n);continue;}if(errno==EINTR)continue;if(errno==EAGAIN||errno==EWOULDBLOCK){close_port();return false;}close_port();return false;}return true;}
bool VescBoard::send_forwarded(std::uint8_t id,const std::vector<std::uint8_t>&p){std::vector<std::uint8_t>x{COMM_FORWARD_CAN,id};x.insert(x.end(),p.begin(),p.end());return send_payload(x);}

bool VescBoard::request_platform_info(){return send_payload({COMM_CUSTOM_APP_DATA,HB_MAGIC0,HB_MAGIC1,HB_VERSION,HB_GET_PLATFORM_INFO});}
bool VescBoard::request_values(bool right){std::vector<std::uint8_t> p{COMM_GET_VALUES_SELECTIVE};be32(p,VALUE_MASK);return right?send_forwarded(RIGHT_ID,p):send_payload(p);}
bool VescBoard::request_steering_cal(){return send_payload({COMM_CUSTOM_APP_DATA,HB_MAGIC0,HB_MAGIC1,HB_VERSION,HB_GET_STEERING_CAL});}
bool VescBoard::set_rpm(std::int32_t e,bool right){std::vector<std::uint8_t>p{COMM_SET_RPM};be32(p,static_cast<std::uint32_t>(e));return right?send_forwarded(RIGHT_ID,p):send_payload(p);}
bool VescBoard::set_steering_deg(double deg){deg=std::clamp(deg,-30.0,30.0);std::vector<std::uint8_t>p{COMM_CUSTOM_APP_DATA,HB_MAGIC0,HB_MAGIC1,HB_VERSION,HB_SET_STEERING_DEG};be32(p,static_cast<std::uint32_t>(std::lround(deg*1000.0)));return send_payload(p);}
bool VescBoard::send_alive(bool right){std::vector<std::uint8_t>p{COMM_ALIVE};return right?send_forwarded(RIGHT_ID,p):send_payload(p);}

bool VescBoard::values_fresh(bool right,double max_age_s)const{const auto&v=values_[right?1:0];return v.valid&&std::chrono::duration<double>(std::chrono::steady_clock::now()-v.stamp).count()<=max_age_s;}
bool VescBoard::steering_cal_fresh(double max_age_s)const{return steering_cal_.valid&&std::chrono::duration<double>(std::chrono::steady_clock::now()-steering_cal_.stamp).count()<=max_age_s;}
bool VescBoard::timed_out()const{return fd_>=0&&std::chrono::steady_clock::now()-last_rx_>std::chrono::seconds(2);}

bool VescBoard::poll(){if(fd_<0){open_port();return fd_>=0;}std::uint8_t b[512];for(;;){ssize_t n=::read(fd_,b,sizeof(b));if(n>0){feed(b,static_cast<std::size_t>(n));continue;}if(n==0||errno==EAGAIN||errno==EWOULDBLOCK)break;if(errno==EINTR)continue;close_port();return false;}if(timed_out()){close_port();return false;}return true;}

void VescBoard::feed(const std::uint8_t*d,std::size_t n){rx_.insert(rx_.end(),d,d+n);while(!rx_.empty()){auto it=std::find_if(rx_.begin(),rx_.end(),[](std::uint8_t x){return x==2||x==3||x==4;});if(it==rx_.end()){rx_.clear();break;}rx_.erase(rx_.begin(),it);if(rx_.empty())break;std::size_t h=rx_[0];if(rx_.size()<h)break;std::size_t len=0;if(h==2)len=rx_[1];else if(h==3){len=(rx_[1]<<8)|rx_[2];if(len<255){rx_.erase(rx_.begin());continue;}}else{len=(static_cast<std::size_t>(rx_[1])<<16)|(static_cast<std::size_t>(rx_[2])<<8)|rx_[3];if(len<65535){rx_.erase(rx_.begin());continue;}}const std::size_t total=h+len+3;if(rx_.size()<total)break;if(rx_[total-1]!=3){rx_.erase(rx_.begin());continue;}const auto c=static_cast<std::uint16_t>((rx_[h+len]<<8)|rx_[h+len+1]);if(c==crc16(rx_.data()+h,len)){std::vector<std::uint8_t>p(rx_.begin()+static_cast<std::ptrdiff_t>(h),rx_.begin()+static_cast<std::ptrdiff_t>(h+len));last_rx_=std::chrono::steady_clock::now();process_payload(p);rx_.erase(rx_.begin(),rx_.begin()+static_cast<std::ptrdiff_t>(total));}else rx_.erase(rx_.begin());}}

void VescBoard::process_payload(const std::vector<std::uint8_t>&p){if(p.empty())return;if(p[0]==COMM_GET_VALUES_SELECTIVE&&p.size()>=5){const std::uint32_t mask=u32(p,1);std::size_t i=5;VescValues v;for(int bit=0;bit<22;++bit){if(!(mask&(1u<<bit)))continue;auto need=[&](std::size_t n){return i+n<=p.size();};if(bit==0){if(!need(2))return;v.temp_mos_c=static_cast<std::int16_t>(u16(p,i))/10.0;i+=2;}else if(bit==1){i+=2;}else if(bit==2||bit==3||bit==4||bit==5||bit==7||bit==9||bit==10||bit==11||bit==12||bit==13||bit==14||bit==16||bit==19||bit==20){if(!need(4))return;double x=i32(p,i);i+=4;if(bit==2)v.current_motor_a=x/100.0;else if(bit==3)v.current_in_a=x/100.0;else if(bit==4)v.id_a=x/100.0;else if(bit==5)v.iq_a=x/100.0;else if(bit==7)v.erpm=x;else if(bit==16)v.position_deg=x/1000000.0;else if(bit==19)v.vd_v=x/1000.0;else if(bit==20)v.vq_v=x/1000.0;}else if(bit==6||bit==8){if(!need(2))return;double x=static_cast<std::int16_t>(u16(p,i));i+=2;if(bit==6)v.duty=x/1000.0;else v.vin_v=x/10.0;}else if(bit==15||bit==17||bit==21){if(!need(1))return;std::uint8_t x=p[i++];if(bit==15)v.fault=x;else if(bit==17)v.vesc_id=x;}else if(bit==18){if(!need(6))return;i+=6;}}
    if(v.vesc_id==1||v.vesc_id==2){auto&dst=values_[v.vesc_id-1];v.valid=true;v.generation=dst.generation+1;v.stamp=std::chrono::steady_clock::now();dst=v;}return;}
  if(p[0]==COMM_CUSTOM_APP_DATA&&p.size()>=6&&p[1]==HB_MAGIC0&&p[2]==HB_MAGIC1&&p[3]==HB_VERSION){const std::uint8_t op=p[4],status=p[5];if(status!=0)return;if(op==HB_GET_PLATFORM_INFO&&p.size()>=48){role_=static_cast<VescRole>(p[46]);role_caps_=p[47];return;}if(op==HB_GET_STEERING_CAL&&p.size()>=23){SteeringCal c;const auto flags=p[6];c.calibrated=flags&1;c.homed=flags&2;c.synced=flags&4;c.logical_inverted=flags&8;c.span=i32(p,7);c.position=i32(p,11);c.target=i32(p,15);c.steering_deg=i32(p,19)/1000.0;std::size_t i=23;if(p.size()>=i+8){c.sensor_port=p[i++];c.sensor_mode=p[i++];c.encoder_configured=p[i++]!=0;c.fault=p[i++];c.encoder_raw=u32(p,i);i+=4;}if(p.size()>=i+8){c.safe_span=i32(p,i);i+=4;c.pos360_deg=i32(p,i)/1000.0;}c.valid=true;c.stamp=std::chrono::steady_clock::now();steering_cal_=c;return;}}
  if(p[0]==COMM_FW_VERSION){return;}}
}  // namespace esc
