#include <stdlib.h>

#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

#include <vecIntConverter/main.hpp>
#include <extspread/extspread.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/algorithm/string.hpp>

#include <scxmlparser/scxmlparser.h>

#include <QScxmlStateMachine>

#include <chrono>
#include <iostream>
#include <fstream>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>

#include "twb_to_tf_bridge/src/enum.pb.h"
#include "twb_to_tf_bridge/src/loc.pb.h"
#include "twb_to_tf_bridge/src/pose.pb.h"
#include "twb_to_tf_bridge/src/rotation.pb.h"
#include "twb_to_tf_bridge/src/shapes.pb.h"
#include "twb_to_tf_bridge/src/vertex.pb.h"
#include "twb_to_tf_bridge/src/TwbTracking.h"

using namespace std;
using namespace rsb;
using namespace rsb::patterns;
using namespace muroxConverter;
using namespace boost;

bool noamiro;
bool norobot;
bool sendAmiroCmd;
bool debugfile;
bool setParamBool;
bool checktwb;
bool grasperror = false;

static std::string extMotorCmdScope = "/graspma/amiro/targetpostion";
static std::string extMotorCmdAnswer = "/graspma/amiro/positionanswer";
static std::string checkObjectScope = "/graspma/amiro/manipulation";
static std::string checkObjectAnswer = "/graspma/amiro/manipulationanswer";
static std::string initAmiroScope = "/graspma/amiro/init";
static std::string initAmiroAnswer = "/graspma/amiro/initanswer";
static std::string extRobotCmdScope = "/graspma/intern/cmd";
static std::string extRobotCmdAnswer = "/graspma/intern/answer";


bool twb;
bool error = false;
static std::string checkTWB = "/tracking/cam3/";
boost::shared_ptr<std::string> twbposition;

boost::shared_ptr<std::string> emptyParam(new std::string("empty"));

boost::shared_ptr<std::string> robotAnswer;
boost::shared_ptr<std::string> setParam = emptyParam;
boost::shared_ptr<std::string> objectAmiro;



Informer<std::vector<int> >::Ptr amiroInformer;
Informer<std::string>::Ptr robotInformer;
Informer<std::string >::Ptr checkObjectInformer;
Informer<std::string>::Ptr amiroInitInformer;


boost::shared_ptr<std::string> RobotCmdGrasp(new std::string("grasp"));
boost::shared_ptr<std::string> Init(new std::string("init"));
boost::shared_ptr<std::string> RobotCmdPlace(new std::string("place"));
boost::shared_ptr<std::string> RobotCmdReset(new std::string("reset"));
boost::shared_ptr<std::string> outOfRange(new std::string("AmiroPosOutOfRange"));
boost::shared_ptr<std::string> testMode(new std::string("NoRobotTestMode"));
boost::shared_ptr<std::string> timeout(new std::string("timeout"));
boost::shared_ptr<std::string> setParamFailed(new std::string("setParamFailed"));
boost::shared_ptr<std::string> grasperrorAnswer(new std::string("grasperror"));
boost::shared_ptr<std::string> reset(new std::string("reset"));
boost::shared_ptr<std::string> checkobject(new std::string("checkobject"));

boost::shared_ptr<std::vector<int> > drivecommand(new std::vector<int>(3, 0));

std::string extSpreadHost = "localhost";
std::string extSpreadPort = "4823";
std::string outputfilename = "benchmark_log.csv";
std::string debugoutputfilename = "debug_log.csv";
std::string inputfilename = "/home/nils/graspbenchmarking/laptop/include/scxmlparser/example.scxml";
std::string setParamValue = "";
std::string rosbag = "";
std::string additionalanswer = "";
std::string resultrobot = "";

std::chrono::steady_clock::time_point timebegin;
std::chrono::steady_clock::time_point timeend;
int counter[5] = {0, 0, 0, 0, 0};
ofstream outputfile;
ofstream debugoutputfile;

twbTrackingProcess::TrackingObject object;

double currentCounter;
double allCounter;

//  y
//  |               o=0
//  |               |
//  |_____x  o=90 __|

struct Grasp {
    int id;
    std::string resultPlace;
    std::string resultGrasp;
    int x;
    int y;
    int o;
};

typedef vector<Grasp> Grasps;
Grasps grasps;

void sendDrivePos();
void boostoptions(int argc, char **argv);
void writeToCSV();
void writeDebugCSV();
void counterFunc();
bool checkAmiroPosition();
void waitForEnter();
void savePositionTWB();
void writeTWB();

ScxmlParser scxmlParser;
Command action;

int main(int argc, char **argv) {

    boostoptions(argc, argv);
    scxmlParser.init(argc, argv, inputfilename);
    string robotname = scxmlParser.getRobotName();
    if (robotname != "" && outputfilename == "benchmark_log.csv") {
        outputfilename = "benchmark_" + robotname + "_log.csv";
    }
    outputfile.open(outputfilename);
    if (debugfile) {
        debugoutputfile.open(debugoutputfilename);
        debugoutputfile << "Statename ,Action ,ID ,ActionID ,Amiro Position X ,Amiro Position Y ,Amiro Orientation ,Duration \n";
        if (noamiro && norobot) {
            outputfile << "No real Robot Data, only for Tests.\n";
        }
    }
    if (noamiro && norobot) {
        outputfile << "No real Robot Data, only for Tests.\n";
    }
    if (twb) {
        debugoutputfile.open(debugoutputfilename);
        debugoutputfile << "Timestamp ,Statename ,Action ,ID ,ActionID ,Amiro Position X ,Amiro Position Y ,Amiro Orientation ,Duration \n";
    }

    Factory& factory = getFactory();

    ParticipantConfig extSpreadConfig = getextspreadconfig(factory, extSpreadHost, extSpreadPort);

    RemoteServerPtr remoteServer = factory.createRemoteServer("/graspma/intern");

    // Register converters
    twbTrackingProcess::registerTracking();

    boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
    rsb::converter::ProtocolBufferConverter<twbTracking::proto::Object>::Ptr converterObject(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Object>);
    converterRepository<std::string>()->registerConverter(converterVecInt);
    converterRepository<std::string>()->registerConverter(converterObject);

    amiroInformer = factory.createInformer<std::vector<int> > (extMotorCmdScope, extSpreadConfig);
    checkObjectInformer = factory.createInformer<std::string> (checkObjectScope, extSpreadConfig);
    amiroInitInformer = factory.createInformer<std::string> (initAmiroScope, extSpreadConfig);


    ListenerPtr listenerMotorCmd = factory.createListener(extMotorCmdAnswer, extSpreadConfig);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueAmiroDriveAnswer(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerMotorCmd->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueAmiroDriveAnswer)));

    ListenerPtr listenerobjectAmiro = factory.createListener(checkObjectAnswer, extSpreadConfig);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueAmiroObjectAnswer(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerobjectAmiro->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueAmiroObjectAnswer)));

    ListenerPtr listenerAmiroInit = factory.createListener(initAmiroAnswer, extSpreadConfig);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueAmiroInit(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerAmiroInit->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueAmiroInit)));

    // prepare rsb listener for tracking data
    rsb::ListenerPtr trackingListener = factory.createListener(checkTWB); // twbTrackingProcess::getTrackingRSBConfig()
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr < twbTracking::proto::ObjectList>>>trackingQueue(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr < twbTracking::proto::ObjectList >> (1));
    trackingListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::ObjectList>(trackingQueue)));

    std::vector<twbTrackingProcess::TrackingObject> trackingObjectList;

    if (sendAmiroCmd) {
        while (true) {
            printf("Enter target location: x y o\n");
            int x, y, o;
            cin >> x >> y >> o;
            action.amiro_o_pos = x;
            action.amiro_y_pos = y;
            action.amiro_o_pos = o;
            sendDrivePos();
            while (queueAmiroDriveAnswer->empty()) {
                usleep(10000);
            }
            robotAnswer = queueAmiroDriveAnswer->pop();
        }
    }

    if(checktwb){
    outputfile << "ID1,X,Y,Theta,ID2,X,Y,Theta,ID3,X,Y,Theta,ID4,X,Y,Theta\n";
    while(true){
        for(int i=0; i<200;i++)
        {
        trackingObjectList = twbTrackingProcess::getNextTrackingObjects(trackingQueue);
        if (trackingObjectList.size() != 0) {
            if (trackingObjectList.at(0).id >= 0) {
                for (int idx = 0; idx < trackingObjectList.size(); idx++) {
                    object = trackingObjectList.at(idx);
                    outputfile << object.id << "," << object.pos.x << "," << object.pos.y << "," << object.pos.theta << ",";
                                    }
                outputfile << "\n";
                outputfile.flush();
            }
        }
        usleep(5000);
        }
        outputfile <<"\n\n\n\n";
        waitForEnter();
    }} else {
        outputfile << "Timestamp, State ,Action ,Object Position X ,Object Position Y ,Object Orientation ,Time ,Max-Time ,Grasp Type ,Robot Pose ,Manipulation Object ,Additional Parameter, Rosbag Name, Set Parameter, Result AMiRo, Result Robot, Result, Additional Return\n";
        outputfile.flush();
    }

    printf("Start init.\n");
    amiroInitInformer->publish(Init);
    try {
        robotAnswer = remoteServer->call<std::string>("robotCmd", Init, 50);
    } catch (rsc::threading::FutureTimeoutException e) {
        printf("Init failed\n");
    }

    while((*robotAnswer != "init_done") || queueAmiroInit->empty())
    {
        sleep(1);
        printf("wait for init.\n");
    }

    printf("Start main routine.\n");
    while (true) {
        action = scxmlParser.getNextAction();
        if (action.action == "end") {
            printf("all actions done. End programm.\n");
            outputfile.close();
            debugoutputfile.close();
            exit(1);
        }
        scxmlParser.printCmd(action);
        outputfile.flush();
        if (noamiro && norobot) {
            printf("Write Files to CSV for Testing.\n");
            timebegin = std::chrono::steady_clock::now();
            usleep(10000);
            robotAnswer = testMode;
            timeend = std::chrono::steady_clock::now();
            writeToCSV();
            if (debugfile) {
                writeDebugCSV();
            }
        }
        rosbag = "";
        if ((action.action == "drive" || action.action == "driveback") && !noamiro) {
            printf("Amiro.\n");
            timebegin = std::chrono::steady_clock::now();
            if (checkAmiroPosition()) {
                sendDrivePos();
                while (queueAmiroDriveAnswer->empty()) {
                    usleep(10000);
                }
                robotAnswer = queueAmiroDriveAnswer->pop();
            } else {
                printf("Robot can't drive there, Position out off Range.\n");
                robotAnswer = outOfRange;
            }
            timeend = std::chrono::steady_clock::now();

            //SAVE TWB DATA
            if (twb) {
                sleep(1);
                trackingObjectList = twbTrackingProcess::getNextTrackingObjects(trackingQueue);
                if (trackingObjectList.size() != 0) {
                    if (trackingObjectList.at(0).id >= 0) {
                        for (int idx = 0; idx < trackingObjectList.size(); idx++) {
                            object = trackingObjectList.at(idx);
                            std::cout << "MarkerX:" << object.pos.x << "MarkerY: " << object.pos.y << object.pos.theta << std::endl;
                        }
                    }
                }
                writeTWB();
                debugoutputfile.flush();
            }
            if (debugfile) {
                writeDebugCSV();
            }

        }
        if (action.action == "grasp" && !norobot) {
            setParam = emptyParam;
            grasperror = false;
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream stringstream;
            stringstream << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%X:%M:%S");
            std::string startcmd = stringstream.str() + ":start";
            std::string stopcmd = stringstream.str() + ":stop";
            rosbag = stringstream.str();
            try{
                remoteServer->call<std::string>("startRecording", boost::shared_ptr<std::string>(new string(startcmd)),10);
            } catch(rsc::threading::FutureTimeoutException e){
                printf("recording failed!\n");
            }
            printf("SetParam\n");
            try {
                std::string graspstype = "grasptype=" + action.grasptype;
                setParam = remoteServer->call<std::string>("setParameter", boost::shared_ptr<std::string>(new string(graspstype)),10);
            } catch(rsc::threading::FutureTimeoutException e){
                if(*setParam != "setParamDone")
                {
                    printf("Set Grasptype failed.\n");
                    setParamBool = false;
                }
            }
            if(action.robot_default_pose != ""){
                try {
                    std::string defaultpose = "robot_default_pose=" + action.robot_default_pose;
                    setParam = remoteServer->call<std::string>("setParameter", boost::shared_ptr<std::string>(new string(defaultpose)),10);
                } catch(rsc::threading::FutureTimeoutException e){
                    if(*setParam != "setParamDone")
                    {
                        printf("Set Grasptype failed.\n");
                        setParamBool = false;
                    }
                }
            }

            if(action.additionalParameter != ""){
            QString additionalParam = QString::fromStdString(action.additionalParameter);
            QStringList parameter = additionalParam.split(";");
            for (int i = 0; i < parameter.size(); i++) {
                try {
                    QString test = parameter.at(i);
                    std::string para = test.toStdString();
                    cout << "Set: " << para << std::endl;
                    setParam = remoteServer->call<std::string>("setParameter", boost::shared_ptr<std::string>(new string(para)),10);
                } catch (rsc::threading::FutureTimeoutException e) {
                    printf("Set Param failed.\n");
                    setParam = setParamFailed;
                }
                string answer = *setParam;
                cout << "Set Para answer: " << answer << std::endl;
                if (*setParam != "setParamDone") {
                    setParamBool = false;
                }
            }
            }
            checkObjectInformer->publish(reset);
            timebegin = std::chrono::steady_clock::now();
            printf("Grasp.\n");
            try {
                robotAnswer = remoteServer->call<std::string>("robotCmd", RobotCmdGrasp, action.max_grasp_time);
            } catch (rsc::threading::FutureTimeoutException e) {
                printf("Grasp failed");
                error = true;
                grasperror = true;

                robotAnswer = timeout;
            }
            timeend = std::chrono::steady_clock::now();
            checkObjectInformer->publish(checkobject);
            while(queueAmiroObjectAnswer->empty())
            {
                usleep(10000);
            }
            objectAmiro = queueAmiroObjectAnswer->pop();
            if(*objectAmiro == "near_object")
            {
                printf("Amiro detected an object.");
                error = true;
                grasperror = true;

            }
            std::string answerhelp = *robotAnswer;
            if (!(answerhelp.find( "grasp_success") != std::string::npos)) {
                error = true;
                grasperror = true;
            }

            resultrobot = answerhelp.substr(0,answerhelp.find_first_of(":"));
            additionalanswer = answerhelp.substr(answerhelp.find_first_of(":") + 1);
            if(additionalanswer == resultrobot){
                additionalanswer = "";
            }

            try{
                remoteServer->call<std::string>("startRecording", boost::shared_ptr<std::string>(new string(stopcmd)),10);
            } catch(rsc::threading::FutureTimeoutException e){
                printf("recording failed!\n");
            }
            writeToCSV();
            if (debugfile) {
                writeDebugCSV();
            }

        }

        if (action.action == "place" && !norobot) {
            setParam = emptyParam;
            if(grasperror)
            {
                printf("Can't place, because there was a grasp error!\n");
                robotAnswer = grasperrorAnswer;
                auto t = std::time(nullptr);
                auto tm = *std::localtime(&t);
                outputfile << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ","
                        << action.statename << ","
                        << action.action << ","
                        << action.amiro_x_pos << ","
                        << action.amiro_y_pos << ","
                        << action.amiro_o_pos << ","
                        << "" << ","
                        << action.max_place_time << ","
                        << action.grasptype << ","
                        << action.robot_default_pose << ","
                        << action.object << ","
                        << action.additionalParameter << ",,,,,,"
                        << "Can't place, there was a grasp error!\n";

            } else {
            printf("Place.\n");
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream stringstream;
            stringstream << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%X:%M:%S");
            std::string startcmd = stringstream.str() + ":start";
            std::string stopcmd = stringstream.str() + ":stop";
            rosbag = stringstream.str();
            try{
                remoteServer->call<std::string>("startRecording", boost::shared_ptr<std::string>(new string(startcmd)),10);
            } catch(rsc::threading::FutureTimeoutException e){
                printf("recording failed!\n");
            }
            checkObjectInformer->publish(reset);
            timebegin = std::chrono::steady_clock::now();
            try {
                robotAnswer = remoteServer->call<std::string>("robotCmd", RobotCmdPlace, action.max_place_time);
            } catch (rsc::threading::FutureTimeoutException e) {
                printf("Place failed");
                error = true;
                robotAnswer = timeout;
            }

            std::string answerhelp = *robotAnswer;
            if (!((answerhelp.find("place_success") != std::string::npos) || (answerhelp.find("place_noresult") != std::string::npos))) {
                error = true;
            }
            resultrobot = answerhelp.substr(0,answerhelp.find_first_of(":"));
            additionalanswer = answerhelp.substr(answerhelp.find_first_of(":") + 1);
            if(additionalanswer == resultrobot){
                additionalanswer = "";
            }

            timeend = std::chrono::steady_clock::now();
            checkObjectInformer->publish(checkobject);
            while(queueAmiroObjectAnswer->empty())
            {
                usleep(10000);
            }
            objectAmiro = queueAmiroObjectAnswer->pop();
            if(*objectAmiro == "near_object")
            {
                printf("Amiro didnt recognize the Place, or detected an object.");
                error = true;
            }
            try{
                remoteServer->call<std::string>("startRecording", boost::shared_ptr<std::string>(new string(stopcmd)),10);
            } catch(rsc::threading::FutureTimeoutException e){
                printf("recording failed!\n");
            }
            writeToCSV();
            if (debugfile) {
                writeDebugCSV();
            }
            }
        }
        if(error)
        {
            printf("Error detected\n");
            printf("The robot will open his hand and go to default position.\n");
            printf("Please place the Object at the AMiRo\n");
            waitForEnter();
            try {
            robotAnswer = remoteServer->call<std::string>("robotCmd", RobotCmdReset, action.max_grasp_time);
            } catch (rsc::threading::FutureTimeoutException e) {
                printf("Reset failed, can't solve the error.");
                robotAnswer = timeout;
                waitForEnter();
            }
            if(*robotAnswer != "reset_success")
            {
                printf("cant reset alone. Please help.\n");
                waitForEnter();
            }
            error = false;
        }
    }
    return EXIT_SUCCESS;
}

void writeToCSV() {

    std::chrono::duration<double, std::milli> duration = timeend - timebegin;
    double durationtime = duration.count();
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    if (setParamBool) {
        setParamValue = "done";
    } else {
        setParamValue = "failed";
    }
    std::string x_pos = "invalid position";
    std::string y_pos = "invalid position";
    std::string o_pos = "invalid position";

    if(checkAmiroPosition())
    {
        x_pos = std::to_string(action.amiro_x_pos);
        y_pos = std::to_string(action.amiro_y_pos);
        o_pos = std::to_string(action.amiro_o_pos);
    }
    std::string result = "";
    if(((resultrobot.find("success") != std::string::npos) ||(resultrobot.find("noresult") != std::string::npos)) && *objectAmiro == "success"){
        result = "success";
    }else if (*objectAmiro != "success") {
        result = "fail";
    } else {
        result = resultrobot;
    }

    //"Timestamp ,Statename, Action ,Object Position X ,Object Position Y ,Object Orientation ,Time, Max-Time, Grasp Type, Robot Pose, Additional Parameter,
    //Rosbag Name, Set Parameter, Result AMiRo, Result Robot, Additional Return, Result\n"
    if (action.action == "grasp") {
        outputfile << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ","
                << action.statename << ","
                << action.action << ","
                << x_pos << ","
                << y_pos << ","
                << o_pos << ","
                << durationtime << ","
                << action.max_grasp_time << ","
                << action.grasptype << ","
                << action.robot_default_pose << ","
                << action.object << ","
                << action.additionalParameter << ","
                << rosbag << ","
                << setParamValue << ","
                << *objectAmiro << ","
                << resultrobot << ","
                << result << ","
                << additionalanswer << "\n";
    } else if (action.action == "place") {
        outputfile << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ","
                << action.statename << ","
                << action.action << ","
                << x_pos << ","
                << y_pos << ","
                << o_pos << ","
                << durationtime << ","
                << action.max_place_time << ","
                << action.grasptype << ","
                << action.robot_default_pose << ","
                << action.object << ","
                << action.additionalParameter << ","
                << rosbag << ","
                << setParamValue << ","
                << *objectAmiro << ","
                << resultrobot << ","
                << result << ","
                << additionalanswer << "\n";
    }
    setParamBool = true;
    additionalanswer = "";
    resultrobot = "";
}

void writeTWB() {

    counterFunc();
    std::chrono::duration<double, std::milli> duration = timeend - timebegin;
    double durationtime = duration.count();
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    debugoutputfile << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ","
            << action.statename << ","
            << action.action << ","
            << allCounter << ","
            << currentCounter << ","
            << object.pos.x << ","
            << object.pos.y << ","
            << object.pos.theta << ","
            << durationtime << "\n"; //TODO DATA
}

void writeDebugCSV() {
    counterFunc();
    std::chrono::duration<double, std::milli> duration = timeend - timebegin;
    double durationtime = duration.count();
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    outputfile << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << "," << action.statename << "," << action.action << "," << allCounter << "," << currentCounter << "," << action.amiro_x_pos << "," <<
            action.amiro_y_pos << "," << action.amiro_o_pos << "," << durationtime << "," << *robotAnswer << "\n";

    //<< "," << std::chrono::system_clock::now() << ","
}

void waitForEnter() {
    printf("Press ENTER to continue...");
    while (cin.get() != '\n');
}

void counterFunc() {
    if (action.action == "drive") {
        counter[0] = counter[0] + 1;
        currentCounter = counter[0];
    }
    if (action.action == "driveback") {
        counter[1] = counter[1] + 1;
        currentCounter = counter[1];

    }
    if (action.action == "grasp") {
        counter[2] = counter[2] + 1;
        currentCounter = counter[2];

    }
    if (action.action == "place") {
        counter[3] = counter[3] + 1;
        currentCounter = counter[3];

    }
    counter[4] = counter[0] + counter[1] + counter[2] + counter [3];
    allCounter = counter[4];
}

bool checkAmiroPosition() {
    if (action.amiro_x_pos <= action.amiro_x_pos_max && action.amiro_x_pos >= action.amiro_x_pos_min &&
            action.amiro_y_pos <= action.amiro_y_pos_max && action.amiro_y_pos >= action.amiro_y_pos_min) {
        return true;
    } else {
        printf("Invalid Target Position.\n");
        return false;
    }
}

void sendDrivePos() {
    if (action.action == "driveback") {
        drivecommand->at(0) = 0;
        drivecommand->at(1) = 0;
        drivecommand->at(2) = 0;
        printf("Drive back\n");
        amiroInformer->publish(drivecommand);
    } else {
        drivecommand->at(0) = action.amiro_x_pos;
        drivecommand->at(1) = action.amiro_y_pos;
        drivecommand->at(2) = action.amiro_o_pos;
        printf("Drive to Position %d %d %d\n", action.amiro_x_pos, action.amiro_y_pos, action.amiro_o_pos);
        amiroInformer->publish(drivecommand);
    }
}

void boostoptions(int argc, char **argv) {
    boost::program_options::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("extSpreadHost", boost::program_options::value<std::string> (&extSpreadHost), "IP of the extern spread server. Default localhost")
            ("extSpreadPort", boost::program_options::value<std::string> (&extSpreadPort), "Port of the extern spread server. Default 4823")
            ("outputFileName", boost::program_options::value<std::string> (&outputfilename), "Output name for the results or for the generated XML if generateXML is true."
            " Default output.xml")
            ("inputFileName", boost::program_options::value<std::string> (&inputfilename), "Name of the XML Input File. Default ist example.scxml.")
            ("noamiro", "Mode without Amiro.")
            ("norobot", "Mode without Robot.")
            ("sendAmiroCmd", "Send Drive Cmds with Terminal Input.")
            ("debug", "Generates a Debug csv File with more Informations.")
            ("twb", "save position from twb after drive cmd.")
            ("checktwb", "check twb, skill is blocking");

    boost::program_options::positional_options_description p;
    p.add("value", 1);

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

    if (vm.count("help")) {
        std::cout << options << "\n";
        exit(1);
    }
    if (vm.count("noamiro")) {
        noamiro = true;
    }
    if (vm.count("norobot")) {
        norobot = true;
    }
    if (vm.count("sendAmiroCmd")) {
        sendAmiroCmd = true;
    }
    if (vm.count("debug")) {
        debugfile = true;
    }
    if (vm.count("twb")) {
        twb = true;
    }
    if(vm.count("checktwb")){
        checktwb = true;
    }

    boost::program_options::notify(vm);
}
