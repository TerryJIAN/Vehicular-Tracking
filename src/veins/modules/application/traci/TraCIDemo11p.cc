//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/traci/TraCIDemo11p.h"

#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"

#include <string>
#include <iostream>
#include <sstream>

std::ofstream myfile;

using namespace Veins;

Define_Module(Veins::TraCIDemo11p);

void TraCIDemo11p::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        predict =true; //域測旗標

        dis_Module =false;
        V_dis = 20;

        wake = false;
        sentGeocast = false;
        findTarget = false;
        off_BSM = false;
        miss_flag =false;
        Geo_only = false; //MISS geocast 唯一選擇

        im_target = false;
        target_Flag = true;
        c = 0;
        time_c =0;

        A =8;
        B =5;
        find = false;

    }
}
void TraCIDemo11p::finish()
{
    DemoBaseApplLayer::finish();
    // statistics recording goes here
}


void TraCIDemo11p::onWSA(DemoServiceAdvertisment* wsa)
{
//    if (currentSubscribedServiceId == -1) {
//        mac->changeServiceChannel(static_cast<Channel>(wsa->getTargetChannel()));
//        currentSubscribedServiceId = wsa->getPsid();
//        if (currentOfferedServiceId != wsa->getPsid()) {
//            stopService();
//            startService(static_cast<Channel>(wsa->getTargetChannel()), wsa->getPsid(), "Mirrored Traffic Service");
//        }
//    }
}

void TraCIDemo11p::onWSM(BaseFrame1609_4* frame)
{
    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);
//    findHost()->getDisplayString().updateWith("r=60,green");
//    if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getDemoData(), 9999);
//    if (!sentMessage) {
//        sentMessage = true;
//        // repeat the received traffic update once in 2 seconds plus some random delay
//        wsm->setSenderAddress(myId);
//        wsm->setSerial(3);
//        scheduleAt(simTime() + 2 + uniform(0.01, 0.2), wsm->dup());
//    }
//    find = true;
    Coord send_p = wsm->getSender_Position();
    Coord t_p=wsm->getTarget_Position();
    double t_s;
    if (wsm->getTarget_Speed().length()<10){
        t_s=10;
    }else{
        t_s= wsm->getTarget_Speed().length();
    }

    if (im_target && wsm->getGeoKey()==1){ //目標
        if(target_Flag){
        TraCIDemo11pMessage* wsm1 = new TraCIDemo11pMessage();
        populateWSM(wsm1,wsm->getSenderAddress());
        wsm1->setGeoKey(2);
        wsm1->setFindTime(wsm->getFindTime());
        wsm1->setSenderAddress(myId);
        wsm1->setTarget_Position(t_p); //record the target position, that is center of circle(Geocast center)
        wsm1->setSender_Position(self_P);
        wsm1->setTarget_Speed(wsm->getTarget_Speed());//per unit range
        sendDown(wsm1);
        //std::cout<<"Im_Target"<< "\n";
        target_Flag= false;
        }
    }
    else if(im_target == false && wsm->getGeoKey()==2){ //參與車輛
        TraCIDemo11pMessage* wsm2 = new TraCIDemo11pMessage();
        findHost()->getDisplayString().updateWith("r=60,DeepSkyBlue");
        populateWSM(wsm2);
        wsm2->setGeoKey(3);
        wsm2->setFindTime(wsm->getFindTime());
        wsm2->setSenderAddress(myId);
        wsm2->setTarget_Position(t_p); //record the target position, that is center of circle(Geocast center)
        wsm2->setSender_Position(self_P);
        wsm2->setTarget_Speed(wsm->getTarget_Speed());//per unit range
        sendDown(wsm2);
        if (predict){
        //    std::cout<<"HAVE_Key"<< "\n";
            std::cout<<"MyID: "<<myId<< "\n";
            std::cout<<t_p-wsm->getTarget_Speed()*A<< "\n";//預測
            std::cout<<simTime()<< "\n";

            myfile.open ("DATA.txt",std::ios_base::app);
            myfile << "MyID: "<<myId<< "\n";
            myfile <<"Find:"<<t_p-wsm->getTarget_Speed()*A<< "\n";//預測
            myfile <<  simTime()<< "\n";
            myfile.close();


            findtime = wsm->getFindTime();
            miss_flag = true;
            lastT_P = t_p-wsm->getTarget_Speed()*A;
        }else{
        //    std::cout<<"HAVE_Key"<< "\n";
            std::cout<<"MyID: "<<myId<< "\n";
            std::cout<<simTime()<< "\n";
            std::cout<<t_p<< "\n";

            myfile.open ("DATA.txt",std::ios_base::app);
            myfile <<"MyID: "<<myId<< "\n";
            myfile <<"Find:"<<t_p<< "\n";
            myfile <<simTime()<< "\n";
            myfile.close();
        }
    }else if (wsm->getGeoKey()==3){
        //    simtime_t f_t = wsm->getFindTime();
                ///***************************************************************************/
        if(predict){
                //    std::cout << "###############Receive################\n";
                //    std::cout << "t_s:"<<t_s<<"\n";
                //    std::cout << "###############Receive################\n";
                    if (self_P.distance(t_p)/t_s <= A+B && self_P.distance(t_p)/t_s >= A-B ){
                        //findHost()->getDisplayString().updateWith("r=60,green");
                        if(self_P.distance(t_p) < lastself_P.distance(t_p) ){
                            wake = true;
                            wake_time = simTime();
                            std::cout <<"wake"<<"\n";
                            //stringstream ss = "\"r=" << (t_s)*A << ",blue\"";
                            //string = s.c_str();
                            findHost()->getDisplayString().updateWith("r=20,blue");

                            myfile.open ("DATA.txt",std::ios_base::app);
                            myfile << "wake \n";
                            myfile.close();

                        }
                    }else if (self_P.distance(t_p)/t_s <= A-B){
                        findHost()->getDisplayString().updateWith("r=20,green");
                //        wake = false;
                        off_BSM = true;
                        off_BSM_time = simTime();
                    }

          if(miss_flag && self_P.distance(send_p) <= 200){
               findHost()->getDisplayString().updateWith("r=60,LightPink");
               miss_flag =false;
               findtime = 0;
             }

                    /***************************************************************************/
        }else{
                /***************************************************************************/
                    if(self_P.distance(t_p) <= 200 && self_P.distance(wsm->getSender_Position()) <200)
                    {
                        findHost()->getDisplayString().updateWith("r=60,green");
                        wake_time = simTime();
                        wake = true;
                        std::cout <<"Rival_wake"<<"\n";

                        myfile.open ("DATA.txt",std::ios_base::app);
                        myfile << "Rival_wake \n";
                        myfile.close();
                    }
                /***************************************************************************/
        }


    }
//-----------------------------MISS------------------------------------------------------
    if(wsm->getGeoKey()==4){
        if(self_P.distance(wsm->getTarget_Position())>200 ){//圓外
            if(self_P.distance(wsm->getTarget_Position()) < wsm->getSender_Position().distance(wsm->getTarget_Position())){
            findHost()->getDisplayString().updateWith("r=30,DeepSkyBlue");
            TraCIDemo11pMessage* wsm3 = new TraCIDemo11pMessage();
            populateWSM(wsm3,wsm->getSenderAddress());
            wsm3->setGeoKey(5);
            wsm3->setFindTime(0);
            wsm3->setSenderAddress(myId);
            wsm3->setTarget_Position(wsm->getTarget_Position()); //record the target position, that is center of circle(Geocast center)
            wsm3->setSender_Position(self_P);
    //        wsm3->setTarget_Speed(null);//per unit range
            sendDelayedDown(wsm3->dup(),uniform(0.3, 0.8));
            }
        }else if (self_P.distance(wsm->getTarget_Position()) <=200 ){//圓內
            findHost()->getDisplayString().updateWith("r=30,DeepSkyBlue");
            TraCIDemo11pMessage* wsm4 = new TraCIDemo11pMessage();
            populateWSM(wsm4,wsm->getSenderAddress());
            wsm4->setGeoKey(5);
            wsm4->setFindTime(0);
            wsm4->setSenderAddress(myId);
            wsm4->setTarget_Position(wsm->getTarget_Position()); //record the target position, that is center of circle(Geocast center)
            wsm4->setSender_Position(self_P);
    //        wsm3->setTarget_Speed(null);//per unit range
            sendDelayedDown(wsm4->dup(),uniform(0.01, 0.3));
        }
    }

   if(wsm->getGeoKey()==5 && Geo_only){ //族頭
        TraCIDemo11pMessage* wsm5 = new TraCIDemo11pMessage();
        populateWSM(wsm5,wsm->getSenderAddress());
        wsm5->setGeoKey(6);
        wsm5->setFindTime(0);
        wsm5->setSenderAddress(myId);
        wsm5->setTarget_Position(wsm->getTarget_Position()); //record the target position, that is center of circle(Geocast center)
        wsm5->setSender_Position(self_P);
//      wsm3->setTarget_Speed(null);//per unit range
        sendDown(wsm5);
        Geo_only = false;
    }
   if(wsm->getGeoKey()==6){
       Geo_only = true;
       if(self_P.distance(wsm->getTarget_Position())>200 ){
           TraCIDemo11pMessage* wsm6 = new TraCIDemo11pMessage();
           populateWSM(wsm6);
           wsm6->setGeoKey(4);
           wsm6->setFindTime(0);
           wsm6->setSenderAddress(myId);
           wsm6->setTarget_Position(wsm->getTarget_Position()); //record the target position, that is center of circle(Geocast center)
           wsm6->setSender_Position(self_P);
   //        wsm->setTarget_Speed(null);//per unit range
           sendDown(wsm6);
       }else if (self_P.distance(wsm->getTarget_Position()) <=200 ){
           TraCIDemo11pMessage* wsm7 = new TraCIDemo11pMessage();
           populateWSM(wsm7);
           wsm7->setGeoKey(7);
           wsm7->setFindTime(0);
           wsm7->setSenderAddress(myId);
           wsm7->setTarget_Position(wsm->getTarget_Position()); //record the target position, that is center of circle(Geocast center)
           wsm7->setSender_Position(self_P);
   //        wsm->setTarget_Speed(null);//per unit range
           sendDown(wsm7);
           Geo_only = false;
       }
   }
   if( wsm->getGeoKey()==7 && self_P.distance(wsm->getTarget_Position()) > 100 && self_P.distance(wsm->getSender_Position()) <200  &&  wake == false)
   {
       TraCIDemo11pMessage* wsm8 = new TraCIDemo11pMessage();
       populateWSM(wsm8);
       wsm8->setGeoKey(8);
       wsm8->setFindTime(0);
       wsm8->setSenderAddress(myId);
       wsm8->setTarget_Position(wsm->getTarget_Position()); //record the target position, that is center of circle(Geocast center)
       wsm8->setSender_Position(self_P);
       sendDelayedDown(wsm8->dup(),uniform(0.01, 0.2));
         //        wsm->setTarget_Speed(null);//per unit range
       findHost()->getDisplayString().updateWith("r=30,red");
       wake_time = simTime();
       wake = true;
       std::cout <<"MISS_wake"<<"\n";

       myfile.open ("DATA.txt",std::ios_base::app);
       myfile <<"MISS_wake"<<"\n";
       myfile.close();

   }
   if( wsm->getGeoKey()==8 && self_P.distance(wsm->getTarget_Position()) > 200 && self_P.distance(wsm->getSender_Position()) <200 &&  wake == false){
       findHost()->getDisplayString().updateWith("r=40,red");
       wake_time = simTime();
       wake = true;
       std::cout <<"MISS_wake"<<"\n";

       myfile.open ("DATA.txt",std::ios_base::app);
       myfile <<"MISS_wake"<<"\n";
       myfile.close();
   }





}
void TraCIDemo11p::onBSM(DemoSafetyMessage* bsm) //refer to TraCIDemo11p.h, which is same as MyVeinsApp.h
{

    // when receiving an event beacon
    Coord target_p;
/*預測路線與區域*/
    if (predict){
        target_p = bsm->getSenderPos() + (bsm->getSenderSpeed()*A) ;//預測
    }else{
        target_p=bsm->getSenderPos();

    }
    double dis = self_P.distance(bsm->getSenderPos());
    if (off_BSM != true)/*是否發現目標 */
    {
        if(wake == false && dis <= 5 && find != true ){
            sentGeocast = true;
            findHost()->getDisplayString().updateWith("r=60,red");
//            std::cout << "*************************RED*************************\n";
//            std::cout << "myId : " << myId << "\n";
//            std::cout << "bsm->getSenderPos() : " << bsm->getSenderPos() << "\n";
//            std::cout << "simTime : " << simTime() << "\n";
//            std::cout << "*****************************************************\n";
            off_BSM = true;
            off_BSM_time = simTime();
        }

        else if (wake == true &&  dis <= V_dis){
            if (dis_Module && dis > 20){
                if (uniform (0.01,1) > dis/V_dis){
                  sentGeocast = true;
                  findHost()->getDisplayString().updateWith("r=60,black");
    //            std::cout << "------------------------black------------------------\n";
    //            std::cout << "myId : " << myId << "\n";
    //            std::cout << "bsm->getSenderPos() : " << bsm->getSenderPos() << "\n";
    //            std::cout << "simTime : " << simTime() << "\n";
    //            std::cout << "-----------------------------------------------------\n";
                  off_BSM = true;
                  off_BSM_time = simTime();
                }
            }else
            {
              sentGeocast = true;
              findHost()->getDisplayString().updateWith("r=60,black");
//            std::cout << "------------------------black------------------------\n";
//            std::cout << "myId : " << myId << "\n";
//            std::cout << "bsm->getSenderPos() : " << bsm->getSenderPos() << "\n";
//            std::cout << "simTime : " << simTime() << "\n";
//            std::cout << "-----------------------------------------------------\n";
              off_BSM = true;
              off_BSM_time = simTime();
            }

        }
        else{
            sentGeocast = false;
        }
    }


/*發送 取得key */
    if (sentGeocast == true){
        TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
        populateWSM(wsm,bsm->getSenderAddress());
        wsm->setGeoKey(1);
        wsm->setFindTime(simTime());
        wsm->setSenderAddress(myId);
        wsm->setTarget_Position(target_p); //record the target position, that is center of circle(Geocast center)
        wsm->setSender_Position(self_P);
        wsm->setTarget_Speed(bsm->getSenderSpeed());//per unit range
//        sendDown(wsm);
        sendDelayedDown(wsm->dup(),uniform(0.01, 0.3));
        sentGeocast = false;
    }
//        double range =bsm->getSenderSpeed().length()*A;  // per unit range
        //record data, such as event(overSpeed -> 5 score)
        //wsm->setDemoData(mobility->getRoadId().c_str());
//        wsm->setFindTime(simTime());

//        sendDelayedDown(wsm->dup(), 2 + uniform(0.01, 0.2));

//    //copy paste
//    if (dataOnSch) {
//        startService(Channel::sch2, 42, "Traffic Information Service");
//        // started service and server advertising, schedule message to self to send later
//        scheduleAt(computeAsynchronousSendingTime(1, ChannelType::service), wsm);
//    }
//    else {
        // send right away on CCH, because channel switching is disabled
//        sendDown(wsm);
        //sendDelayedDown(wsm->dup(), 2 + uniform(0.01, 0.2));
//    }

}

void TraCIDemo11p::handleSelfMsg(cMessage* msg)
{
      DemoBaseApplLayer::handleSelfMsg(msg);
      findHost()->getDisplayString().updateWith("r=200,yellow");
      im_target = true;

      if (target_Flag == false){
          c = c+1;
          if(c==2){
              target_Flag = true;
              c=0;
          }
      }
      if (time_c%2 == 0 ){
          myfile.open ("DATA.txt",std::ios_base::app);
          myfile <<"Target_P:"<<self_P<< "\n";
          myfile <<simTime()<< "\n";
          myfile.close();
      }
      time_c = time_c+1;


//    if (TraCIDemo11pMessage* wsm = dynamic_cast<TraCIDemo11pMessage*>(msg)) {
        // send this message on the service channel until the counter is 3 or higher.
        // this code only runs when channel switching is enabled
        //findHost()->getDisplayString().updateWith("r=60,green");
//        sendDown(wsm->dup());
//        wsm->setSerial(wsm->getSerial() + 1);
//        if (wsm->getSerial() >= 3) {
//            // stop service advertisements
//            stopService();
//            delete (wsm);
//        }
//        else {
//            scheduleAt(simTime() + 1, wsm);
//        }
//    }
//    else {
//    }
}

void TraCIDemo11p::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);

    if (self_P != mobility->getPositionAt(simTime())){
        lastself_P = self_P;
    }self_P = mobility->getPositionAt(simTime());

    if (wake && simTime() - wake_time > 30){
        wake = false;
        findHost()->getDisplayString().updateWith("r=5,blue");
    }
    if (off_BSM && simTime() - off_BSM_time > 5){
        off_BSM = false;

    }

    if (miss_flag ){
        if(simTime()-findtime > 2*A){
            std::cout <<"MISS"<<"\n";
            if(self_P.distance(lastT_P)>200){
                std::cout <<"MyID: "<<myId<< "\n";
                std::cout <<"MISS_Time: "<<simTime()<< "\n";
                miss_flag = false;
                Geo_only = true;

                TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
                findHost()->getDisplayString().updateWith("r=60,red");
                populateWSM(wsm);
                wsm->setGeoKey(4);
                wsm->setFindTime(0);
                wsm->setSenderAddress(myId);
                wsm->setTarget_Position(lastT_P); //record the target position, that is center of circle(Geocast center)
                wsm->setSender_Position(self_P);
        //        wsm->setTarget_Speed(null);//per unit range
                sendDown(wsm);

            }else if(self_P.distance(lastT_P)<=200){
                std::cout <<"MyID: "<<myId<< "\n";
                std::cout <<"MISS_Time: "<<simTime()<< "\n";
                miss_flag = false;
                Geo_only = false;
                TraCIDemo11pMessage* wsm1 = new TraCIDemo11pMessage();
                findHost()->getDisplayString().updateWith("r=60,red");
                populateWSM(wsm1);
                wsm1->setGeoKey(7);
                wsm1->setFindTime(0);
                wsm1->setSenderAddress(myId);
                wsm1->setTarget_Position(lastT_P); //record the target position, that is center of circle(Geocast center)
                wsm1->setSender_Position(self_P);
        //        wsm->setTarget_Speed(null);//per unit range
                sendDown(wsm1);

            }

        }
    }

    if (simTime() - 0 > 824){
 //       find = true;

    }

//    if (Maxid !=0 && Maxdis!=0 && simTime()-wait_time > 2){
//        findHost()->getDisplayString().updateWith("r=30,black");
//        TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
//        populateWSM(wsm);
//        wsm->setSenderAddress(myId);
//        wsm->setReceiveAddress(Maxid);
//        wsm->setRelay(3);
//        wsm->setTarget_Position(Target_P); //record the target position, that is center of circle(Geocast center)
//        wsm->setSender_Position(self_P);
//        wsm->setRangeData(Range);
//        wsm->setFindTime(FindTime);
//        sendDown(wsm);
//        Maxid =0;
//        Maxdis =0;
//        wait_time =0;
//    }
    //findHost()->getDisplayString().parse("display");
    // stopped for for at least 10s?
//    if (mobility->getSpeed() < 1) {
//        if (simTime() - lastDroveAt >= 10 && sentMessage == false) {

//            findHost()->getDisplayString().updateWith("r=60,blue");
//            sentMessage = true;
//
//            TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
//            populateWSM(wsm);
//            wsm->setDemoData(mobility->getRoadId().c_str());
//
//
//            // host is standing still due to crash
//            if (dataOnSch) {
//                startService(Channel::sch2, 42, "Traffic Information Service");
//                // started service and server advertising, schedule message to self to send later
//                scheduleAt(computeAsynchronousSendingTime(1, ChannelType::service), wsm);
//            }
//            else {
//                // send right away on CCH, because channel switching is disabled
//                sendDown(wsm);
//            }
//        }
//    }
//    else {
//        lastDroveAt = simTime();
//    }
}
