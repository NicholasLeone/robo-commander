#include <thread>
#include <pigpiod_if2.h>

#include "utils/utils.h"
#include "robots/swansonV2.h"

using namespace std;

SwansonV2::SwansonV2(int pi){

     this->_pi = pi;
     this->_port = 14500;

     /**************************************************************************
     * LOAD CONFIG FILE
     **************************************************************************/
     // std::map<std::string, float> variables;
     // LoadInitialVariables("../../config/profiles/swanson.config", variables);

     // printf("%d %d %d %d %d %d %d %d\r\n",fr_pwm,fr_dir,fl_pwm,fl_dir,rr_pwm,rr_dir,rl_pwm,rl_dir);

     /**************************************************************************
     * END LOAD CONFIG FILE
     **************************************************************************/

     rc_in = new UDP(_port,NULL);
     claws = new DualClaw(pi);
     claws->reset_encoders();

}


SwansonV2::~SwansonV2(){

     delete rc_in;
     delete claws;

}


void SwansonV2::drive(float v, float w){

     vector<int32_t> cmds = claws->set_speeds(v, w);
     claws->drive(cmds);

}

void SwansonV2::readRC(){
     int i = 0;

     UDP* udp_line = rc_in;
     RC_COMMAND_MSG* data = &controls;

	char* dat = udp_line->read(sizeof(data)+24);
	memcpy(data, &dat[16],sizeof(data)+8);
}


void SwansonV2::updateSensors(){

     claws->update_status();
     claws->update_encoders();

     // printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",_current_pose[0],_current_pose[1],_current_pose[5]);
     // printf("V1, V2, V3, V4:     %.5f   |    %.5f |    %.5f |    %.5f \r\n",spd1,spd2,spd3,spd4);

}
