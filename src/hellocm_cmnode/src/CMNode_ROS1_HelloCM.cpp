/*!
 ******************************************************************************
 **  CarMaker - Version 7.1.2
 **  Vehicle Dynamics Simulation Toolkit
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 *
 * Description:
 * - Prototype/Proof of Concept
 * - Unsupported ROS Example with CarMaker
 * - Structure may change in future!
 * - Change general parameters in Infofile for CMRosIF ("Data/Config/CMRosIFParameters")
 * - Basic communication with or without parameterizable synchronization
 *
 *
 * ToDo:
 * - C++!!!
 * - ROS naming/way/namespaces
 * - parameter: CarMaker read, ROS set by service?
 *   -> ROS parameter mechanism seems better solution!
 * - node/topic/... destruction to allow dynamic load/unload
 *   when TestRun starts instead of initialization at CarMaker startup
 * - New Param_Get() function to read parameters from Infofile
 * - ...
 *
 */


/* CarMaker
 * - include other headers e.g. to access to vehicle data
 *   - e.g. "Vehicle.h" or "Vehicle/Sensor_*.h".
 * - additional headers can be found in "<CMInstallDir>/include/"
 * - see Reference Manual, chapter "User Accessible Quantities" to find some variables
 *   that are already defined in DataDictionary and their corresponding C-Code Name
 */
#include "Log.h"
#include "DataDict.h"
#include "SimCore.h"
#include "InfoUtils.h"

#include "apo.h"
#include "GuiCmd.h"

// #include "Vehicle.h"
// #include "DrivMan.h"
// #include "Vehicle/Sensor_Object.h"

#include "VehicleControl.h"
#include "DrivMan.h"
#include "Vehicle_Control_UDP.h"
extern tUDP_PC UDP_PC;
extern tUDP_Input UDP_Input;

#include "Vehicle.h"
#include "Car/Car.h"
#include "Car/Steering.h"
#include "Vehicle/Sensor_LidarRSI.h"
#include "Vehicle/Sensor_GNav.h"
#include "Vehicle/Sensor_Inertial.h"



#define VLP_16_NUMBER_OF_POINTS 28800
#define VLP_16_POINTS_OF_PACKET 2880
#define VLP_16_NUMBER_OF_PACKET 10

#define OS1_64_NUMBER_OF_POINTS 65536
#define OS1_64_POINTS_OF_PACKET 2900
#define OS1_64_POINTS_OF_LAST_PACKET 1736
#define OS1_64_NUMBER_OF_PACKET 23
#define Max_Intensity 65000

int Lidar_CycleCount;


/* ROS */
#include "cmrosutils/CMRosUtils.h"    /* Node independent templates, ...*/
#include "cmrosutils/CMRosIF_Utils.h" /* Only for CarMaker ROS Node!!! Functions are located in library for CarMaker ROS Interface */
#include "cmrosutils/CMRemote.h"      /* Basic service for CarMaker remote from ROS */

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <angles/angles.h>
#include <hellocm_msgs/GPS_Out.h>
#include <hellocm_msgs/UDP.h>
#include <hellocm_msgs/imu_UDP.h>
#include <hellocm_msgs/sub_udp.h>


/* Following header from external ROS node can be used to get topic/service/... names
 * Other mechanism:
 * 1. Put names manually independently for each node
 * 2. Using command line arguments or launch files and ROS remapping
 * - Doing so, only general message headers are necessary
 */
#if 1
#  include "hellocm/ROS1_HelloCM.h"  /* External ROS Node. Topic name, ... */
#else
#  include <hellocm_msgs/Ext2CM.h>
#  include <hellocm_msgs/CM2Ext.h>
#  include <hellocm_msgs/Init.h>
#endif

/*! String and numerical version of this Node
 *  - String:    e.g. <Major>.<Minor>.<Patch>
 *  - Numerical: e.g. <nDigitsMajor><2DigitsMinor><2DigitsPatch>
 */
#define CMNODE_VERSION "0.8.0"
#define CMNODE_NUMVER  800


/* NDEBUG is set in CarMaker Makefile/MakeDefs in OPT_CFLAGS */
#if !defined NDEBUG
#  warning "Debug options are enabled!"
#  define DBLOG LOG
#else
#  define DBLOG(...)
#endif

/* Not beautiful but consistent with external ROS Node
 * where ROS_INFO is used (implicit newline)*/
# define LOG(frmt, ...)  Log(frmt "\n", ##__VA_ARGS__)


/* General switches for CarMaker ROS Node */
typedef enum tCMNode_Mode {
    CMNode_Mode_Disabled  = 0,  /*!< Node is disabled. e.g. don't publish. */
    CMNode_Mode_Default   = 1,  /*!< Node is enabled, spinOnce is used  */
    CMNode_Mode_Threaded  = 2   /*!< Node is enabled, spin in parallel thread
                                     - Messages are received all the time
                                     - Data is updated at defined position, e.g. *_In()
                                     - Currently not implemented! */
} tCMNode_Mode;



/* Managing synchronization between CarMaker Node and other ROS nodes */
typedef enum tCMNode_SyncMode {
    CMNode_SyncMode_Disabled  = 0, /*!< No synchronization on CarMaker side */
    CMNode_SyncMode_Tpc       = 1  /*!< Buffer messages or Spin until external Topics are received */
} tCMNode_SyncMode;



/* Global struct for this Node */
static struct {
    unsigned long  CycleNoRel;  /*!< CarMaker relative cycle number, e.g. since start of TestRun */

    struct {
        double         Duration;      /*!< Time spent for synchronization task */
        int            nCycles;       /*!< Number of cycles in synchronization loop */
        int            CyclePrepDone; /*!< Last cycle when preparation was done */
        int            CycleJobDone;  /*!< Last cycle when job was done */
        double         SynthDelay;    /*!< Synthetic delay in seconds provided to external node to check sync */
    } Sync; /*!< Synchronization related information */

    struct {
        int CycleNo;      /*!< Cycle number of external ROS Node (only for information) */

        /* For debugging */
        int            CycleLastOut;   /*!< Cycle number when Topic was published */
        int            CycleLastIn;    /*!< Cycle number when Topic from external ROS Node was received */
        int            CycleLastFlush; /*!< Cycle number when data from external ROS Node was provided to model */
    } Model; /*!< Model related information. ROS side! */

    struct {
        struct {
            tRosIF_TpcSub<hellocm_msgs::Ext2CM> Ext2CM; /* For this example also used for Synchronization */
            tRosIF_TpcSub<hellocm_msgs::sub_udp> sub_udp; /* For this example also used for Synchronization */
                        
        } Sub; /*!< Topics to be subscribed */

        struct {
            tRosIF_TpcPub<hellocm_msgs::CM2Ext> CM2Ext;
			tRosIF_TpcPub<sensor_msgs::PointCloud> Lidar_VLP_left; 
			tRosIF_TpcPub<sensor_msgs::PointCloud> Lidar_VLP_right; 
			tRosIF_TpcPub<sensor_msgs::PointCloud> Lidar_VLP_front; 
			tRosIF_TpcPub<sensor_msgs::Imu> Imu; 
			tRosIF_TpcPub<hellocm_msgs::GPS_Out> GPS_Out;
            tRosIF_TpcPub<hellocm_msgs::UDP> UDP;
            tRosIF_TpcPub<hellocm_msgs::imu_UDP> imu_UDP;
            tRosIF_TpcPub<nav_msgs::Odometry> Odometry;

			
            /*!< CarMaker can be working as ROS Time Server providing simulation time
             *   starting at 0 for each TestRun */
            tRosIF_TpcPub<rosgraph_msgs::Clock> Clock;
        } Pub; /*!< Topics to be published */
    } Topics; /*!< ROS Topics used by this Node */

    struct {
        /*!< Initialization/Preparation of external ROS Node e.g. when simulation starts */
        tRosIF_Srv<hellocm_msgs::Init>    Init;
        tRosIF_Srv<cmrosutils::CMRemote>  CMRemote; // Trial
    } Services; /*!< ROS Services used by this Node (client and server)*/
	
	struct {
        boost::shared_ptr<tf2_ros::TransformBroadcaster> br;
        boost::shared_ptr<tf2_ros::StaticTransformBroadcaster> st_br;
		
        geometry_msgs::TransformStamped Lidar_VLP_left;
		geometry_msgs::TransformStamped Lidar_VLP_right;
		geometry_msgs::TransformStamped Lidar_VLP_front;
		
    } TF;
	

    struct {
        int               QueuePub;     /*!< Queue size for Publishers */
        int               QueueSub;     /*!< Queue size for Subscribers */
        int               nCyclesClock; /*!< Number of cycles publishing /clock topic.
                                             CycleTime should be multiple of this value */
        tCMNode_Mode      Mode;
        tCMNode_SyncMode  SyncMode;
        double            SyncTimeMax;  /* Maximum Synchronization time */

        tRosIF_Cfg        Ros;
    } Cfg; /*!< General configuration for this Node */
	
	struct {
        double *BeamTable;
        int rows = 0;
    } LidarRSI_VLP_LEFT, LidarRSI_VLP_RIGHT,  LidarRSI_VLP_FRONT;

} CMNode;



/*!
 * Description:
 * - Callback for ROS Topic published by external ROS Node
 *
 */
static void
cmnode_HelloCM_CB_TpcIn (const hellocm_msgs::Ext2CM::ConstPtr &msg)
{
    /* Process message only if receive is expected */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled)
	return;
    
    int rv;
    auto in = &CMNode.Topics.Sub.Ext2CM;

    /* Update receive buffer
     * - No lock for spinOnce necessary?
     */
    in->Msg.header  = msg->header;
    in->Msg.time    = msg->time;
    in->Msg.cycleno = msg->cycleno;

    /* Stopping simulation is only necessary when synchronization is activated */
    if (CMNode.Cfg.SyncMode == CMNode_SyncMode_Tpc && (rv = CMCRJob_DoPrep_SetDone(in->Job, CMNode.CycleNoRel)) != CMCRJob_RV_Success) {
	LogErrF(EC_Sim, "CMNode: Error on DoPrep_SetDone for Job '%s'! rv=%s", CMCRJob_GetName(in->Job), CMCRJob_RVStr(rv));
    }

    /* Remember cycle for debugging */
    CMNode.Model.CycleLastIn = CMNode.CycleNoRel;


    LOG("%s (CMSimTime=%.3fs): External Node is in cycle %lu, Time=%.3fs, Stamp=%.3fs, SeqID=%d",
	    ros::this_node::getName().c_str(), SimCore.Time,
	    in->Msg.cycleno, msg->time.toSec(), in->Msg.header.stamp.toSec(), in->Msg.header.seq);

}

static void
cmnode_sub_udp_TpcIn (const hellocm_msgs::sub_udp::ConstPtr &msg)
{
    /* Process message only if receive is expected */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled)
	return;
    
    int rv;
    auto in = &CMNode.Topics.Sub.sub_udp;

    /* Update receive buffer
     * - No lock for spinOnce necessary?
     */
    in->Msg.SteeringWheel  = msg->SteeringWheel;
    in->Msg.Ax  = msg->Ax;
    in->Msg.GearNo  = msg->GearNo;
    in->Msg.VC_SwitchOn  = msg->VC_SwitchOn;

    /* Stopping simulation is only necessary when synchronization is activated */
    if (CMNode.Cfg.SyncMode == CMNode_SyncMode_Tpc && (rv = CMCRJob_DoPrep_SetDone(in->Job, CMNode.CycleNoRel)) != CMCRJob_RV_Success) {
	LogErrF(EC_Sim, "CMNode: Error on DoPrep_SetDone for Job '%s'! rv=%s", CMCRJob_GetName(in->Job), CMCRJob_RVStr(rv));
    }

    /* Remember cycle for debugging */
    CMNode.Model.CycleLastIn = CMNode.CycleNoRel;


    // LOG("%s (CMSimTime=%.3fs): External Node is in cycle %lu, Time=%.3fs, Stamp=%.3fs, SeqID=%d",
	//     ros::this_node::getName().c_str(), SimCore.Time,
	//     in->Msg.cycleno, msg->time.toSec(), in->Msg.header.stamp.toSec(), in->Msg.header.seq);

}



/*!
 * Description:
 * - Exemplary Service Callback for CarMaker Remote using ROS
 * - e.g. via rqt Service Caller or terminal "rosservice call ..."
 *
 *
 */
static bool
cmnode_HelloCM_CB_SrvCMRemote(cmrosutils::CMRemote::Request &req,
	cmrosutils::CMRemote::Response &resp)
{

    int rv = -2;
    char sbuf[512];

    LOG("%s: Service '%s' was triggered with type='%s', msg='%s', data='%s'",
	    ros::this_node::getName().c_str(),
	    CMNode.Services.CMRemote.Srv.getService().c_str(),
	    req.type.c_str(), req.msg.c_str(), req.data.c_str());


    /* Commands to CarMaker GUI
     * - Tcl commands!
     * - More information see "ProgrammersGuide"
     */
    if (strcasecmp("guicmd", req.type.c_str()) == 0) {
	/* Default: Evaluate command sent with message */
	if (strcasecmp("eval", req.msg.c_str()) == 0) {
	    /* e.g. data = 'LoadTestRun CMRosIF/AdaptiveCruiseControl; StartSim' */
	    rv = GuiCmd_Eval(req.data.c_str());
	} else {
	    if (strcasecmp("start", req.msg.c_str()) == 0) {
		if (req.data.length() == 0)
		    rv = GuiCmd_Eval("LoadTestRun CMRosIF/AdaptiveCruiseControl; StartSim");
		else {
		    sprintf(sbuf, "%s; StartSim", req.data.c_str());
		    rv = GuiCmd_Eval(sbuf);
		}
	    }
	    if (strcasecmp("stop", req.msg.c_str()) == 0)
		rv = GuiCmd_Eval("StopSim");
	}


	/* Commands directly to CarMaker Executable
	 * - Warning:
	 *   - Information normally provided by CarMaker GUI might be missing
	 */
    } else if (strcasecmp("cmd", req.type.c_str()) == 0) {
	if (strcasecmp("start", req.msg.c_str()) == 0) {
	    if (req.data.length() == 0) {
		/* Most strings are normally provided by CarMaker GUI */
		SimStart(NULL, ros::this_node::getName().c_str(),
			"CMRosIF/AdaptiveCruiseControl", NULL, NULL);
	    } else {
		/* Most strings are normally provided by CarMaker GUI */
		SimStart(NULL, ros::this_node::getName().c_str(),
			req.data.c_str(), NULL, NULL);
	    }
	}
	if (strcasecmp("stop", req.msg.c_str()) == 0)
	    SimStop2(0);
	rv = 0;
    }

    resp.res = rv;
    return true;
}



/*****************************************************************************/
/**********          C-Code for interfacing with CarMaker!          **********/
/*****************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Get versions from shared library
 * - Set the returned Version to 0 if there is no dependency!
 * - Compatibility check should be done by calling procedure
 *   as early as possible(e.g. before CMRosIF_CMNode_Init()
 *
 * Arguments:
 * - CMRosIFVer = CMRosIF shared library version (User defined)
 *                - Initially filled with version of CMRosIF management library
 * - CMNumVer   = CarMaker version used for shared library at compile time (normally CM_NUMVER)
 *                - Initially filled with version of CMRosIF management library
 * - RosVersion = ROS version used for shared library at compile time (normally ROS_VERSION)
 *                - Initially filled with version requested by CMRosIF management library (0 if no request)
 *
 */
int
CMRosIF_CMNode_GetVersion (unsigned long *CMRosIFCMNodeNumVer,
                           unsigned long *CMNumVer,
			   unsigned long *RosNumVer)
{

    *CMRosIFCMNodeNumVer = CMNODE_NUMVER;
    *CMNumVer            = CM_NUMVER;
    *RosNumVer           = ROS_VERSION;

    return 0;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Basic Initialization
 * - e.g. create ROS Node, subscriptions, ...
 * - Return values
 *   - "rv <  0" = Error at initialization, CarMaker executable will stop
 *   - "rv >= 0" = Everything OK, CarMaker executable will continue
 *
 * Arguments:
 * - Argc/Argv  = Arguments normally provided to ROS executable are not be provided
 *                to CM executable directly, but can be set in Infofile for CMRosIF
 *                with key "Node.Args" in "Data/Config/CMRosIFParameters"
 *
 * - CMNodeName = Default CarMaker Node name
 *                - Can be parameterized in Infofile for CMRosIF
 *                - Final node name might be different (argument remapping, ...)
 *
 * - Inf        = Handle to CarMaker Infofile with parameters for this interface
 *                - Please note that pointer may change, e.g. before TestRun begins
 *
 * ToDo:
 * - Possible to create/initialize node/... before each TestRun start instead of CM startup?
 * - New Param_Get() function to read parameters from Infofile
 */
int
CMRosIF_CMNode_Init (int Argc, char **Argv, char *CMNodeName, struct tInfos *Inf)
{

    int rv;
    bool rvb                = false;
    char sbuf[512]          = "";
    char keybuf[256]        = "";
    ros::NodeHandlePtr node = NULL;
    ros::V_string names;


    LOG("Initialize CarMaker ROS Node");
    LOG("  -> Node Version = %05d", CMNODE_NUMVER);
    LOG("  -> ROS Version  = %05d", ROS_VERSION);
    LOG("  -> CM Version   = %05d", CM_NUMVER);

    /* ROS initialization. Name of Node might be different after remapping! */
    if (ros::isInitialized() == false) {
	/* "Remapping arguments" functionality (launchfiles, ...)? */
	ros::init(Argc, Argv, CMNodeName);
    } else {
	//node.reset(); ToDo!
    }

    if (ros::master::check() == false) {
	LogErrF(EC_Init, "Can't contact ROS Master!\n Start roscore or run launch file e.g. via Extras->CMRosIF\n");
	ros::shutdown();
	return -1;
    }

    /* Node specific */
    CMNode.Cfg.Ros.Node = ros::NodeHandlePtr(boost::make_shared<ros::NodeHandle>());
    node                = CMNode.Cfg.Ros.Node;

    /* Publish specific */
    CMNode.Cfg.QueuePub  = iGetIntOpt(Inf, "Node.QueuePub", 1000); /* ToDo: Influence of queue length relevant? */

    /* Prepare the node to provide simulation time. CarMaker will be /clock server */
    strcpy(sbuf, "/use_sim_time");

    if ((rv = node->hasParam(sbuf)) == true) {
	node->getParam(sbuf, rvb);
	LOG("  -> Has param '%s' with value '%d'", sbuf, rvb);
    }

    /* Additional switch to provide simulation Time */
    strcpy(keybuf, "Node.UseSimTime");

    if ((rv = iGetIntOpt(Inf, keybuf, 1)) > 0) {
	/* Parameter must be set before other nodes start
	 * - set parameter outside to be independent from execution order?
	 */
	LOG("  -> Provide simulation time!");
	node->setParam("/use_sim_time", true); /* enable parameter if not already done */

	CMNode.Cfg.nCyclesClock  = iGetIntOpt(Inf, "Node.nCyclesClock", 1000);

	strcpy(sbuf, "/clock");
	LOG("    -> Publish '%s' every %dms", sbuf, CMNode.Cfg.nCyclesClock);
	CMNode.Topics.Pub.Clock.Pub  = node->advertise<rosgraph_msgs::Clock>(sbuf, CMNode.Cfg.QueuePub);


	/* ToDo: Necessary/Possible to ensure /clock is zeroed? */
	CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
	CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
    } else {
	LOG("  -> Don't provide simulation time!");
	CMNode.Cfg.nCyclesClock  = 0;
    }

    strcpy(sbuf, hellocm::tpc_in_name.c_str() /*! Opposite in/out compared to external node */);
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.CM2Ext.Pub         = node->advertise<hellocm_msgs::CM2Ext>(sbuf, CMNode.Cfg.QueuePub);
    CMNode.Topics.Pub.CM2Ext.Job         = CMCRJob_Create("CM2Ext");
    CMNode.Topics.Pub.CM2Ext.CycleTime   = 10;
    CMNode.Topics.Pub.CM2Ext.CycleOffset = 0;
	
	strcpy(sbuf, "/pointcloud/vlp_left");
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.Lidar_VLP_left.Pub         = node->advertise<sensor_msgs::PointCloud>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
    CMNode.Topics.Pub.Lidar_VLP_left.Job         = CMCRJob_Create("pointcloud/vlp_left");

	strcpy(sbuf, "/pointcloud/vlp_right");
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.Lidar_VLP_right.Pub         = node->advertise<sensor_msgs::PointCloud>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
    CMNode.Topics.Pub.Lidar_VLP_right.Job         = CMCRJob_Create("pointcloud/vlp_right");

	strcpy(sbuf, "/pointcloud/vlp_front");
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.Lidar_VLP_front.Pub         = node->advertise<sensor_msgs::PointCloud>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
    CMNode.Topics.Pub.Lidar_VLP_front.Job         = CMCRJob_Create("pointcloud/vlp_front");

	strcpy(sbuf, "/gps_out");
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.GPS_Out.Pub           = node->advertise<hellocm_msgs::GPS_Out>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
    CMNode.Topics.Pub.GPS_Out.Job           = CMCRJob_Create("gps_out");

    //create car_udp pubulisher 
	strcpy(sbuf, "/udp");
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.UDP.Pub           = node->advertise<hellocm_msgs::UDP>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
    CMNode.Topics.Pub.UDP.Job           = CMCRJob_Create("udp");
    CMNode.Topics.Pub.UDP.CycleTime   = 10;
    CMNode.Topics.Pub.UDP.CycleOffset = 0;

    //create imu_senser_udp pubulisher
	strcpy(sbuf, "/imu_udp");
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.imu_UDP.Pub           = node->advertise<hellocm_msgs::imu_UDP>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
    CMNode.Topics.Pub.imu_UDP.Job           = CMCRJob_Create("imu_udp");
    CMNode.Topics.Pub.imu_UDP.CycleTime   = 10;
    CMNode.Topics.Pub.imu_UDP.CycleOffset = 0;

    //create ros_imu pubulisher
    strcpy(sbuf, "/imu"); /*! Opposite in/out compared to external node */
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.Imu.Pub         = node->advertise<sensor_msgs::Imu>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
    CMNode.Topics.Pub.Imu.Job         = CMCRJob_Create("imu");
    CMNode.Topics.Pub.Imu.CycleTime   = 10;
    CMNode.Topics.Pub.Imu.CycleOffset = 0;

	
	strcpy(sbuf, "/Odometry");
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.Odometry.Pub         = node->advertise<nav_msgs::Odometry>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
    CMNode.Topics.Pub.Odometry.Job         = CMCRJob_Create("Odometry");
    CMNode.Topics.Pub.Odometry.CycleTime   = 50;
    CMNode.Topics.Pub.Odometry.CycleOffset = 0;





	
	CMNode.TF.br = boost::make_shared<tf2_ros::TransformBroadcaster>();
    CMNode.TF.st_br = boost::make_shared<tf2_ros::StaticTransformBroadcaster>();
	

    /* Subscribe specific */
    CMNode.Cfg.QueueSub  = iGetIntOpt(Inf, "Node.QueueSub", 1); /* ToDo: Effect of queue length for subscriber? */


    strcpy(sbuf, hellocm::tpc_out_name.c_str() /*! Opposite in/out compared to external node */);
    LOG("  -> Subscribe '%s'", sbuf);
    CMNode.Topics.Sub.Ext2CM.Sub         = node->subscribe(sbuf, CMNode.Cfg.QueueSub, cmnode_HelloCM_CB_TpcIn);
    CMNode.Topics.Sub.Ext2CM.Job         = CMCRJob_Create("Ext2CM_for_Sync");

    /* In this example cycle time might be updated with value of external ROS Node
     * - See CMRosIF_CMNode_TestRun_Start_atBegin() */
    CMNode.Topics.Sub.Ext2CM.CycleTime   = 15000;

    strcpy(sbuf, "/sub_udp");
    LOG("  -> Subscribe '%s'", sbuf);
    CMNode.Topics.Sub.sub_udp.Sub         = node->subscribe(sbuf, CMNode.Cfg.QueueSub, cmnode_sub_udp_TpcIn);
    CMNode.Topics.Sub.sub_udp.Job         = CMCRJob_Create("sub_udp_for_Sync");

    /* In this example cycle time might be updated with value of external ROS Node
     * - See CMRosIF_CMNode_TestRun_Start_atBegin() */
    CMNode.Topics.Sub.sub_udp.CycleTime   = 15000;



    /* Services */
    strcpy(sbuf, hellocm::srv_init_name.c_str());
    LOG("  -> Service Client '%s'", sbuf);
    CMNode.Services.Init.Clnt = node->serviceClient<hellocm_msgs::Init>(sbuf);


    strcpy(sbuf, "CMRemote");
    LOG("  -> Create Service '%s'", sbuf);
    CMNode.Services.CMRemote.Srv = node->advertiseService(
	    sbuf, cmnode_HelloCM_CB_SrvCMRemote);


    /* Print general information after everything is done */
    LOG("Initialization of ROS Node finished!");
    LOG("  -> Node Name = '%s'", ros::this_node::getName().c_str());
    LOG("  -> Namespace = '%s'", ros::this_node::getNamespace().c_str());


    /* Advertised Topics */
    ros::this_node::getAdvertisedTopics(names);
    LOG("  -> Advertised Topics (%lu)", names.size());

    auto it = names.begin();
    for (; it != names.end(); ++it)
	LOG("    -> %s", (*it).c_str());


    /* Subscribed Topics */
    names.clear();
    ros::this_node::getSubscribedTopics(names);
    LOG("  -> Subscribed Topics (%lu)", names.size());
    it = names.begin();
    for (; it != names.end(); ++it)
	LOG("    -> %s",  (*it).c_str());

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Add user specific Quantities for data storage
 *   and visualization to DataDictionary
 * - Called once at program start
 * - no realtime conditions
 *
 */
void
CMRosIF_CMNode_DeclQuants (void)
{

    tDDefault *df = DDefaultCreate("CMRosIF.");

    DDefULong   (df, "CycleNoRel",         "ms", &CMNode.CycleNoRel,               DVA_None);
    DDefInt     (df, "Sync.Cycles",        "-",  &CMNode.Sync.nCycles,             DVA_None);
    DDefDouble  (df, "Sync.Time",          "s",  &CMNode.Sync.Duration,            DVA_None);
    DDefInt     (df, "Sync.CyclePrepDone", "-",  &CMNode.Sync.CyclePrepDone,       DVA_None);
    DDefInt     (df, "Sync.CycleJobDone" , "-",  &CMNode.Sync.CycleJobDone,        DVA_None);
    DDefDouble4 (df, "Sync.SynthDelay",     "s", &CMNode.Sync.SynthDelay,          DVA_IO_In);

    DDefUChar   (df, "Cfg.Mode",           "-",  (unsigned char*)&CMNode.Cfg.Mode, DVA_None);
    DDefInt     (df, "Cfg.nCyclesClock",   "ms", &CMNode.Cfg.nCyclesClock,         DVA_None);
    DDefChar    (df, "Cfg.SyncMode",       "-",  (char*)&CMNode.Cfg.SyncMode,      DVA_None);
    DDefDouble4 (df, "Cfg.SyncTimeMax",    "s",  &CMNode.Cfg.SyncTimeMax,          DVA_IO_In);

    DDefInt     (df, "Mdl.CycleNo",        "-",  &CMNode.Model.CycleNo,            DVA_None);
    DDefInt     (df, "Mdl.CycleLastOut",   "ms", &CMNode.Model.CycleLastOut,       DVA_None);
    DDefInt     (df, "Mdl.CycleLastIn",    "ms", &CMNode.Model.CycleLastIn,        DVA_None);
    DDefInt     (df, "Mdl.CycleLastFlush", "ms", &CMNode.Model.CycleLastFlush,     DVA_None);

    DDefaultDelete(df);
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called when starting a new TestRun
 * - In separate Thread (no realtime conditions)
 * - After standard Infofiles are read in
 * - Return values
 *   - "rv <  0" = Error, TestRun start will be aborted
 *   - "rv >= 0" = Everything OK
 *
 * Arguments:
 * - Inf = CarMaker Infofile for CMRosIF with content after TestRun start
 *         - Please note that the Infofile provided at initialization might have been updated!
 *
 * ToDo:
 * - New Param_Get() function to read parameters from Infofile
 *
 */
int
CMRosIF_CMNode_TestRun_Start_atBegin (struct tInfos *Inf)
{
	srand(time(NULL));
	
	Lidar_CycleCount = 0;
	
	tInfos *Inf_Sensor = nullptr;
    tErrorMsg *err = nullptr;
	
    //Create infofile handle
    Inf_Sensor = InfoNew();
    iRead2(&err, Inf_Sensor, "Data/Sensor/LidarRSI_VLP_16", "");
    CMNode.LidarRSI_VLP_LEFT.BeamTable = (double*)malloc(VLP_16_NUMBER_OF_POINTS * 6 * sizeof(double));
    //Read infofile parameters
    iGetTableOpt(Inf_Sensor, "Beams", CMNode.LidarRSI_VLP_LEFT.BeamTable, VLP_16_NUMBER_OF_POINTS * 6, 6, &CMNode.LidarRSI_VLP_LEFT.rows);
	InfoDelete(Inf_Sensor);
	
    //Create infofile handle
    Inf_Sensor = InfoNew();
    iRead2(&err, Inf_Sensor, "Data/Sensor/LidarRSI_VLP_16", "");
    CMNode.LidarRSI_VLP_RIGHT.BeamTable = (double*)malloc(VLP_16_NUMBER_OF_POINTS * 6 * sizeof(double));
    //Read infofile parameters
    iGetTableOpt(Inf_Sensor, "Beams", CMNode.LidarRSI_VLP_RIGHT.BeamTable, VLP_16_NUMBER_OF_POINTS * 6, 6, &CMNode.LidarRSI_VLP_RIGHT.rows);
	InfoDelete(Inf_Sensor);

    //Create infofile handle
    Inf_Sensor = InfoNew();
    iRead2(&err, Inf_Sensor, "Data/Sensor/LidarRSI_VLP_16", "");
    CMNode.LidarRSI_VLP_FRONT.BeamTable = (double*)malloc(VLP_16_NUMBER_OF_POINTS * 6 * sizeof(double));
    //Read infofile parameters
    iGetTableOpt(Inf_Sensor, "Beams", CMNode.LidarRSI_VLP_FRONT.BeamTable, VLP_16_NUMBER_OF_POINTS * 6, 6, &CMNode.LidarRSI_VLP_FRONT.rows);
	InfoDelete(Inf_Sensor);
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //Read sensor info from Vehicle InfoFile
    tInfos *Inf_Vehicle = nullptr;
    Inf_Vehicle = InfoNew();
    
    const char *FName;
    FName = InfoGetFilename(SimCore.Vhcl.Inf);
	
    Log("FName = %s\n", FName);
	
    int VehicleInfo_Err = iRead2(&err, Inf_Vehicle, FName, "SensorReadCode");
	
    if (VehicleInfo_Err == 0) {
	
        tf2::Quaternion q;
        double* position;
        double* rotation;
		double tmp[3] = {0, 0, 0};
		
		//Lidar Sensor
		position = iGetFixedTableOpt2(Inf_Vehicle, "Sensor.LidarRSI.0.pos", tmp, 3, 1);
		rotation = iGetFixedTableOpt2(Inf_Vehicle, "Sensor.LidarRSI.0.rot", tmp, 3, 1);
		q.setRPY(rotation[0], rotation[1], rotation[2]);
        CMNode.TF.Lidar_VLP_left.transform.rotation = tf2::toMsg(q);
        CMNode.TF.Lidar_VLP_left.transform.translation = tf2::toMsg(tf2::Vector3(position[0], position[1], position[2]));
        CMNode.TF.Lidar_VLP_left.child_frame_id = iGetStrOpt(Inf_Vehicle, "Sensor.LidarRSI.0.name", "LIR00");
        CMNode.TF.Lidar_VLP_left.header.frame_id = iGetStrOpt(Inf_Vehicle, "Sensor.LidarRSI.0.Mounting", "Fr1A");
        CMNode.Topics.Pub.Lidar_VLP_left.CycleTime = iGetIntOpt(Inf_Vehicle, "Sensor.LidarRSI.0.CycleTime", 100);
        CMNode.Topics.Pub.Lidar_VLP_left.CycleOffset = iGetIntOpt(Inf_Vehicle, "Sensor.LidarRSI.0.nCycleOffset", 0);
		
		position = iGetFixedTableOpt2(Inf_Vehicle, "Sensor.LidarRSI.1.pos", tmp, 3, 1);
		rotation = iGetFixedTableOpt2(Inf_Vehicle, "Sensor.LidarRSI.1.rot", tmp, 3, 1);
		q.setRPY(rotation[0], rotation[1], rotation[2]);
        CMNode.TF.Lidar_VLP_right.transform.rotation = tf2::toMsg(q);
        CMNode.TF.Lidar_VLP_right.transform.translation = tf2::toMsg(tf2::Vector3(position[0], position[1], position[2]));
        CMNode.TF.Lidar_VLP_right.child_frame_id = iGetStrOpt(Inf_Vehicle, "Sensor.LidarRSI.1.name", "LIR00");
        CMNode.TF.Lidar_VLP_right.header.frame_id = iGetStrOpt(Inf_Vehicle, "Sensor.LidarRSI.1.Mounting", "Fr1A");
        CMNode.Topics.Pub.Lidar_VLP_right.CycleTime = iGetIntOpt(Inf_Vehicle, "Sensor.LidarRSI.1.CycleTime", 100);
        CMNode.Topics.Pub.Lidar_VLP_right.CycleOffset = iGetIntOpt(Inf_Vehicle, "Sensor.LidarRSI.1.nCycleOffset", 0);
	
		position = iGetFixedTableOpt2(Inf_Vehicle, "Sensor.LidarRSI.2.pos", tmp, 3, 1);
		rotation = iGetFixedTableOpt2(Inf_Vehicle, "Sensor.LidarRSI.2.rot", tmp, 3, 1);
		q.setRPY(rotation[0], rotation[1], rotation[2]);
        CMNode.TF.Lidar_VLP_front.transform.rotation = tf2::toMsg(q);
        CMNode.TF.Lidar_VLP_front.transform.translation = tf2::toMsg(tf2::Vector3(position[0], position[1], position[2]));
        CMNode.TF.Lidar_VLP_front.child_frame_id = iGetStrOpt(Inf_Vehicle, "Sensor.LidarRSI.2.name", "LIR00");
        CMNode.TF.Lidar_VLP_front.header.frame_id = iGetStrOpt(Inf_Vehicle, "Sensor.LidarRSI.2.Mounting", "Fr1A");
        CMNode.Topics.Pub.Lidar_VLP_front.CycleTime = iGetIntOpt(Inf_Vehicle, "Sensor.LidarRSI.2.CycleTime", 100);
        CMNode.Topics.Pub.Lidar_VLP_front.CycleOffset = iGetIntOpt(Inf_Vehicle, "Sensor.LidarRSI.2.nCycleOffset", 0);

		//GNSS
		CMNode.Topics.Pub.GPS_Out.CycleTime     = (int)(1000 / iGetIntOpt(Inf_Vehicle, "Sensor.GNav.UpdRate", 10));
		CMNode.Topics.Pub.GPS_Out.CycleOffset   = iGetIntOpt(Inf_Vehicle, "Sensor.GNav.nCycleOffset", 0);

		
    }
	
    int errCode = InfoDelete(Inf_Vehicle);
    Log("errCode = %d\n", errCode);
    

    /* Node can be disabled via Infofile */
    tCMNode_Mode     *pmode     = &CMNode.Cfg.Mode;
    tCMNode_SyncMode *psyncmode = &CMNode.Cfg.SyncMode;

    if (Inf != NULL) {
	*pmode     =     (tCMNode_Mode)iGetIntOpt(Inf, "Node.Mode",      CMNode_Mode_Disabled);
	*psyncmode = (tCMNode_SyncMode)iGetIntOpt(Inf, "Node.Sync.Mode", CMNode_SyncMode_Disabled);
    }

    if (SimCore.CycleNo == 0 || Inf == NULL || *pmode == CMNode_Mode_Disabled) {
	*pmode = CMNode_Mode_Disabled;
	LOG("CarMaker ROS Node is disabled!");
	return 0;
    }

    char sbuf[512];
    char key[256];
    char *str        = NULL;
    int rv           = 0;
    bool rvb                = false;

    int cycletime           = 0;
    int *pcycletime         = NULL;
    int cycleoff            = 0;
    tCMCRJob *job           = NULL;
    auto srv         = &CMNode.Services.Init;

    LOG("CarMaker ROS Node is enabled! Mode=%d, SyncMode=%d", *pmode, *psyncmode);
    LOG("  -> Node Name = %s", ros::this_node::getName().c_str());


    /* Update synchronization */
    if (*psyncmode != CMNode_SyncMode_Disabled && *psyncmode != CMNode_SyncMode_Tpc) {
	LogErrF(EC_Sim, "CMNode: Invalid synchronization mode '%d'!",*psyncmode);
	*pmode = CMNode_Mode_Disabled;
	return -1;
    }

    CMNode.Cfg.SyncTimeMax = iGetDblOpt(Inf, "Node.Sync.TimeMax", 1.0);


    /* Reset for next cycle */
    CMNode.CycleNoRel           =  0;
    CMNode.Sync.Duration        =  0.0;
    CMNode.Sync.nCycles         = -1;
    CMNode.Sync.CycleJobDone    = -1;
    CMNode.Sync.CyclePrepDone   = -1;
    CMNode.Model.CycleNo        = -1;
    CMNode.Model.CycleLastIn    = -1;
    CMNode.Model.CycleLastOut   = -1;
    CMNode.Model.CycleLastFlush = -1;


    /* Allow an update of the clock only if it was enabled before! */
    if (CMNode.Cfg.nCyclesClock > 0) {
	if ((rv = iGetIntOpt(Inf, "Node.nCyclesClock", 1000)) > 0)
	    CMNode.Cfg.nCyclesClock = rv;
    }

    /* Necessary to ensure /clock is zeroed here?
     * ToDo: Create function? */
    if (CMNode.Cfg.nCyclesClock > 0) {
	LOG("  -> Publish /clock every %dms", CMNode.Cfg.nCyclesClock);
	CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
	CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
    }


    /* Prepare external node for next simulation */
    if (!srv->Clnt.exists()) {
	// ToDo: possible to get update if external ROS Node name changes?
	LogErrF(EC_Sim, "ROS Service is not ready! Please start external ROS Node providing Service '%s'!",
		srv->Clnt.getService().c_str());
	*pmode = CMNode_Mode_Disabled;
	return -1;
    }

    LOG("  -> Send Service Request");

    /* ToDo: Async?*/
    if (!srv->Clnt.call(srv->Msg)) {
	LogErrF(EC_Sim, "ROS Service error!");
	*pmode = CMNode_Mode_Disabled;
	return -1;
    }

    /* Update cycle time with information of external node */
#if 1
    /* Variant 1:
     * - Receiving parameters via ROS Parameter Server
     * - Parameter may be set externally e.g. by other node or arguments to command
     * - ROS parameters are more flexible than ROS services!
     */
    strcpy(sbuf, hellocm::prm_cycletime_name.c_str());
    if ((rv = CMNode.Cfg.Ros.Node->hasParam(sbuf)) == true)
	CMNode.Cfg.Ros.Node->getParam(sbuf, rv);
#else
    /* Variant 2:
     * - Receiving parameters from external Node via Service
     * - Services might be too "static"
     * - Not recommended!
     */
    rv = srv->Msg.response.cycletime;
#endif

    pcycletime = &CMNode.Topics.Sub.Ext2CM.CycleTime;

    if (*pcycletime != rv) {
	LOG("  -> Cycle time of external node changed from %dms to %dms", *pcycletime, rv);
	*pcycletime = rv;
    }


    /* Plausibility check for Cycle Time */
    if (CMNode.Cfg.nCyclesClock > 0 && (*pcycletime < CMNode.Cfg.nCyclesClock
	    || *pcycletime%CMNode.Cfg.nCyclesClock != 0)) {
	    
	LogErrF(EC_Sim, "Ext. ROS Node has invalid cycle time! Expected multiple of %dms but got %dms",
		CMNode.Cfg.nCyclesClock, *pcycletime);
		
	*pmode = CMNode_Mode_Disabled;
	return -1;
    }

    
    
    /* Prepare Jobs for publish and subscribe
     * - Special use case:
     *   - Topic in and Topic out use same cycle time with relative shift!
     */

    /* Start to publish when simulation starts */
    job       = CMNode.Topics.Pub.CM2Ext.Job;
    cycletime = CMNode.Topics.Pub.CM2Ext.CycleTime;
    cycleoff  = CMNode.Topics.Pub.CM2Ext.CycleOffset;

    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);

	job       = CMNode.Topics.Pub.Lidar_VLP_left.Job;
    cycletime = CMNode.Topics.Pub.Lidar_VLP_left.CycleTime;
    cycleoff  = CMNode.Topics.Pub.Lidar_VLP_left.CycleOffset;
    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);
	
	job       = CMNode.Topics.Pub.Lidar_VLP_right.Job;
    cycletime = CMNode.Topics.Pub.Lidar_VLP_right.CycleTime;
    cycleoff  = CMNode.Topics.Pub.Lidar_VLP_right.CycleOffset;
    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);
	
	job       = CMNode.Topics.Pub.Lidar_VLP_front.Job;
    cycletime = CMNode.Topics.Pub.Lidar_VLP_front.CycleTime;
    cycleoff  = CMNode.Topics.Pub.Lidar_VLP_front.CycleOffset;
    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);
	
	job       = CMNode.Topics.Pub.GPS_Out.Job;
    cycletime = CMNode.Topics.Pub.GPS_Out.CycleTime;
    cycleoff  = CMNode.Topics.Pub.GPS_Out.CycleOffset;
	
    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);
    
    //udp_job create
	job       = CMNode.Topics.Pub.UDP.Job;
    cycletime = CMNode.Topics.Pub.UDP.CycleTime;
    cycleoff  = CMNode.Topics.Pub.UDP.CycleOffset;
	
    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

    //imu_senser_udp job_create
	job       = CMNode.Topics.Pub.imu_UDP.Job;
    cycletime = CMNode.Topics.Pub.imu_UDP.CycleTime;
    cycleoff  = CMNode.Topics.Pub.imu_UDP.CycleOffset;
	
    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);


	job       = CMNode.Topics.Pub.Odometry.Job;
    cycletime = CMNode.Topics.Pub.Odometry.CycleTime;
    cycleoff  = CMNode.Topics.Pub.Odometry.CycleOffset;

    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);


    //ros_imu job_create
	job       = CMNode.Topics.Pub.Imu.Job;
    cycletime = CMNode.Topics.Pub.Imu.CycleTime;
    cycleoff  = CMNode.Topics.Pub.Imu.CycleOffset;

    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

	
    /* Synchronization with external node
     * - external node provides cycle time (see service above)
     * - other parameterization methods (e.g. ROS parameter, ...) are possible!
     * - Expect sync Topic are delayed (communication time, ...)
     * - This example shows sync via ROS Timer in external node
     *   - Therefore "/clock" topic needs to be published by CarMaker!
     *   - Other mechanism, e.g. data triggered on external node side
     *     via publishing Topic directly inside subscription callback is also possible!
     * - time=0.0 can't be detected by external node, therefore
     *   first receive needs to start after expected cycle time
     *   of external ROS node
     */

    job       = CMNode.Topics.Sub.Ext2CM.Job;
    cycletime = CMNode.Topics.Sub.Ext2CM.CycleTime;
    cycleoff  = CMNode.Topics.Sub.Ext2CM.CycleOffset = 0; /* No offset allowed if ROS Timer is used for sync!*/


    /* Create the synchronization jobs */
    if (*psyncmode == CMNode_SyncMode_Tpc) {
	CMCRJob_Init(job, cycletime+1 , cycletime, CMCRJob_Mode_Ext);

	LOG("  -> Synchronize on Topic '%s' (cycletime=%d, cycleoff=%d)",
		CMNode.Topics.Sub.Ext2CM.Sub.getTopic().c_str(), cycletime, cycleoff);

    } else
	CMCRJob_Init(job, cycletime+1 , cycletime, CMCRJob_Mode_Default);

    job       = CMNode.Topics.Sub.sub_udp.Job;
    cycletime = CMNode.Topics.Sub.sub_udp.CycleTime;
    cycleoff  = CMNode.Topics.Sub.sub_udp.CycleOffset = 0; /* No offset allowed if ROS Timer is used for sync!*/


    /* Create the synchronization jobs */
    if (*psyncmode == CMNode_SyncMode_Tpc) {
	CMCRJob_Init(job, cycletime+1 , cycletime, CMCRJob_Mode_Ext);

	LOG("  -> Synchronize on Topic '%s' (cycletime=%d, cycleoff=%d)",
		CMNode.Topics.Sub.sub_udp.Sub.getTopic().c_str(), cycletime, cycleoff);

    } else
	CMCRJob_Init(job, cycletime+1 , cycletime, CMCRJob_Mode_Default);




	std::vector<geometry_msgs::TransformStamped> transforms;
	transforms.push_back(CMNode.TF.Lidar_VLP_left);
	transforms.push_back(CMNode.TF.Lidar_VLP_right);
	transforms.push_back(CMNode.TF.Lidar_VLP_front);
	CMNode.TF.st_br->sendTransform(transforms);
	


    LOG("External ROS Node is ready to simulate");

    return 1;
}



/*!
 * ToDo:
 * - Put everything to TestRun_Start_atBegin?
 *
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Repeating call for several CarMaker cycles until return value is 1
 * - May be called even previous return value was 1
 * - See "User.c:User_TestRun_RampUp()"
 *
 */
int
CMRosIF_CMNode_TestRun_RampUp (void)
{

    /* Return immediately if node is disabled */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled)
	return 1;

    /* Put your code here */
    //if (NotReady) return 0;


    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called when TestRun ends (no realtime conditions)
 * - See "User.c:User_TestRun_End()"
 *
 */
int
CMRosIF_CMNode_TestRun_End (void)
{


    /* Put your code here */

    /* Disable after simulation has finished */
    CMNode.Cfg.Mode = CMNode_Mode_Disabled;

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called at very beginning of CarMaker cycle
 * - Process all topics/services using different modes or synchronization modes
 * - See "User.c:User_In()"
 *
 * ToDo:
 * - Additional spin mechanism
 *   - e.g. for HIL
 *   - e.g. spinning in new thread, copying incoming data here, ...
 *
 */
int
CMRosIF_CMNode_In (void)
{

    int rv                   = 0;
    int rx_done              = 0;
    const char *job_name     = NULL;
    tCMCRJob *job            = NULL;
    ros::WallTime tStart     = ros::WallTime::now();
    ros::WallDuration tDelta = ros::WallDuration(0.0);
    CMNode.Sync.nCycles      = 0;
    CMNode.Sync.Duration     = 0.0;

    switch (CMNode.Cfg.Mode) {
	case CMNode_Mode_Disabled:
	    /* Comment next line if no messages/services
	     * shall be processed in disabled Node state
	     */
	    ros::spinOnce();
	    break;

	case CMNode_Mode_Default:

	    if (CMNode.Cfg.SyncMode != CMNode_SyncMode_Tpc) {
                /* Process messages in queue, but do not block */
		ros::spinOnce();

	    } else {
		/* Synchronization based on expected Topics
		 * - Blocking call (process publish and wait for answer)
		 * - Stop simulation if maximum time is exceeded
		 */
		do {
		    ros::spinOnce();

		    /* Only do anything if simulation is running */
		    if (SimCore.State != SCState_Simulate) {
			rx_done = 1;
			break;
		    }

		    rx_done = 0;

		    /* Check all jobs if preparation is done */
		    job      = CMNode.Topics.Sub.Ext2CM.Job;

		    if ((rv = CMCRJob_DoPrep(job, CMNode.CycleNoRel, 0, NULL, NULL)) < CMCRJob_RV_OK) {
			LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s",CMCRJob_GetName(job), CMCRJob_RVStr(rv));
			rx_done = 0;
			break;
		    }

		    /* If job is not done, remember name and prevent loop to finish */
		    job_name = (rv != CMCRJob_RV_DoSomething ? NULL : CMCRJob_GetName(job));
		    rx_done  = rv == CMCRJob_RV_DoNothing ? 1 : 0;

		    if (rx_done == 1)
			break;

		    /* Wait a little that data can arrive. WallTime, NOT ROS time!!!*/
		    ros::WallDuration(0.0).sleep();
		    tDelta = ros::WallTime::now() - tStart;
		    CMNode.Sync.nCycles++;

		} while (ros::ok() && rx_done == 0 && tDelta.toSec() < CMNode.Cfg.SyncTimeMax);

		/* Final calculation to get duration including last cycle before receive */
		tDelta = ros::WallTime::now() - tStart;

		CMNode.Sync.Duration = tDelta.toSec();

		if (rx_done != 1 && CMNode.Cfg.SyncTimeMax > 0.0 && tDelta.toSec() >= CMNode.Cfg.SyncTimeMax)
		    LogErrF(EC_Sim, "CMNode: Synchronization error! tDelta=%.3f, Last invalid Job='%s'\n", tDelta.toSec(), job_name);
	    }

	    break;

	case CMNode_Mode_Threaded:
	    /* ToDo
	     * - Spinning in parallel thread started before
	     * - Lock variables!
	     * - e.g. for HIL
	     */
	    break;

	default:
	    /* Invalid!!! */;
    }

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after driving maneuver calculation
 * - before CMRosIF_CMNode_VehicleControl_Calc()
 * - See "User.c:User_DrivManCalc()"
 */
int
CMRosIF_CMNode_DrivMan_Calc (double dt)
{
    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
	    || SimCore.State != SCState_Simulate)
	return 0;

    /* Put your code here */

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after CMRosIF_CMNode_DrivManCalc
 * - before CMRosIF_CMNode_VehicleControl_Calc()
 * - See "User.c:User_VehicleControl_Calc()"
 */
int
CMRosIF_CMNode_VehicleControl_Calc (double dt)
{
    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
	    || SimCore.State != SCState_Simulate)
	return 0;

    /* Put your code here */
    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after vehicle model has been calculated
 * - See "User.c:User_Calc()"
 */
int
CMRosIF_CMNode_Calc (double dt)
{

    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
	    || SimCore.State != SCState_Simulate)
	return 0;

    /* Put your code here
     * - Update model parameters here?
     * - Do some calculation...
     */

    /* Update model with values from external node only in specific cycle?
     * - This data handling is optionl, but necessary for deterministic behaviour
     * - if synchronization is active, incoming data remains in msg buffer until correct cycle
     */
    int rv;
    auto sync = &CMNode.Topics.Sub.Ext2CM;

    if ((rv = CMCRJob_DoJob(sync->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing
	    && rv != CMCRJob_RV_DoSomething) {
	LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(sync->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {
	/* Something to do in sync cycle? */
	//CMCRJob_Info(in->Job, CMNode.CycleNoRel, "CMNode: Do Something for Sync: ");

	    /* Update model parameters here? */
	CMNode.Model.CycleNo = CMNode.Topics.Sub.Ext2CM.Msg.cycleno;


	/* Remember cycle for debugging */
	CMNode.Sync.CycleJobDone    = CMNode.CycleNoRel;
	CMNode.Model.CycleLastFlush = CMNode.CycleNoRel;
    }

    /* Do some calculation... */

	if (SimCore.State == SCState_Simulate) {
        //Lidar RSI --> PointCloud
        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.Lidar_VLP_left.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "cycleTime: %d, cycleoffset: %d, cycle: %lu", CMNode.Topics.Pub.Lidar_VLP_left.CycleTime, CMNode.Topics.Pub.Lidar_VLP_left.CycleOffset, CMNode.CycleNoRel);
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.Lidar_VLP_left.Job), CMCRJob_RVStr(rv));
        } else if (Lidar_CycleCount % (int)CMNode.Topics.Pub.Lidar_VLP_left.CycleTime == 0) {
            geometry_msgs::Point32 points;
            sensor_msgs::ChannelFloat32 channels;
            channels.name = "intensity";
	
            //clearing vector data to avoid overflows
            CMNode.Topics.Pub.Lidar_VLP_left.Msg.points.clear();
            CMNode.Topics.Pub.Lidar_VLP_left.Msg.channels.clear();
            channels.values.clear();
	
	
            //Lidar Quantity processing
            if (SimCore.State == SCState_Simulate) {
                for (int i = 0; i < LidarRSI[0].nScanPoints; i++) {
	
                    const int beam_id = LidarRSI[0].ScanPoint[i].BeamID;
                    const double azimuth = angles::from_degrees(CMNode.LidarRSI_VLP_LEFT.BeamTable[4*CMNode.LidarRSI_VLP_LEFT.rows + beam_id]);
                    const double elevation = angles::from_degrees(CMNode.LidarRSI_VLP_LEFT.BeamTable[5*CMNode.LidarRSI_VLP_LEFT.rows + beam_id]);
                    const double ray_length = 0.5 * LidarRSI[0].ScanPoint[i].LengthOF; // length of flight is back and forth
	
                    //XYZ-coordinates of scan point
                    points.x = ray_length * cos(elevation) * cos(azimuth);
                    points.y = ray_length * cos(elevation) * sin(azimuth);
                    points.z = ray_length * sin(elevation);
	
                    CMNode.Topics.Pub.Lidar_VLP_left.Msg.points.push_back(points);
                    channels.values.push_back(LidarRSI[0].ScanPoint[i].Intensity);
	
                }
                CMNode.Topics.Pub.Lidar_VLP_left.Msg.channels.push_back(channels);
                CMNode.Topics.Pub.Lidar_VLP_left.Msg.header.frame_id = CMNode.TF.Lidar_VLP_left.child_frame_id;
                CMNode.Topics.Pub.Lidar_VLP_left.Msg.header.stamp = ros::Time(LidarRSI[0].ScanTime);
            }
        }
		
        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.Lidar_VLP_right.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "cycleTime: %d, cycleoffset: %d, cycle: %lu", CMNode.Topics.Pub.Lidar_VLP_right.CycleTime, CMNode.Topics.Pub.Lidar_VLP_right.CycleOffset, CMNode.CycleNoRel);
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.Lidar_VLP_right.Job), CMCRJob_RVStr(rv));
        } else if (Lidar_CycleCount % (int)CMNode.Topics.Pub.Lidar_VLP_right.CycleTime == 0){
            geometry_msgs::Point32 points;
            sensor_msgs::ChannelFloat32 channels;
            channels.name = "intensity";
	
            //clearing vector data to avoid overflows
            CMNode.Topics.Pub.Lidar_VLP_right.Msg.points.clear();
            CMNode.Topics.Pub.Lidar_VLP_right.Msg.channels.clear();
            channels.values.clear();
	
	
            //Lidar Quantity processing
            if (SimCore.State == SCState_Simulate) {
                for (int i = 0; i < LidarRSI[1].nScanPoints; i++) {
	
                    const int beam_id = LidarRSI[1].ScanPoint[i].BeamID;
                    const double azimuth = angles::from_degrees(CMNode.LidarRSI_VLP_RIGHT.BeamTable[4*CMNode.LidarRSI_VLP_RIGHT.rows + beam_id]);
                    const double elevation = angles::from_degrees(CMNode.LidarRSI_VLP_RIGHT.BeamTable[5*CMNode.LidarRSI_VLP_RIGHT.rows + beam_id]);
                    const double ray_length = 0.5 * LidarRSI[1].ScanPoint[i].LengthOF; // length of flight is back and forth
	
                    //XYZ-coordinates of scan point
                    points.x = ray_length * cos(elevation) * cos(azimuth);
                    points.y = ray_length * cos(elevation) * sin(azimuth);
                    points.z = ray_length * sin(elevation);
	
                    CMNode.Topics.Pub.Lidar_VLP_right.Msg.points.push_back(points);
                    channels.values.push_back(LidarRSI[1].ScanPoint[i].Intensity);
	
                }
                CMNode.Topics.Pub.Lidar_VLP_right.Msg.channels.push_back(channels);
                CMNode.Topics.Pub.Lidar_VLP_right.Msg.header.frame_id = CMNode.TF.Lidar_VLP_right.child_frame_id;
                CMNode.Topics.Pub.Lidar_VLP_right.Msg.header.stamp = ros::Time(LidarRSI[1].ScanTime);
            }
        }

        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.Lidar_VLP_front.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "cycleTime: %d, cycleoffset: %d, cycle: %lu", CMNode.Topics.Pub.Lidar_VLP_front.CycleTime, CMNode.Topics.Pub.Lidar_VLP_front.CycleOffset, CMNode.CycleNoRel);
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.Lidar_VLP_front.Job), CMCRJob_RVStr(rv));
        } else if (Lidar_CycleCount % (int)CMNode.Topics.Pub.Lidar_VLP_front.CycleTime == 0){
            geometry_msgs::Point32 points;
            sensor_msgs::ChannelFloat32 channels;
            channels.name = "intensity";
	
            //clearing vector data to avoid overflows
            CMNode.Topics.Pub.Lidar_VLP_front.Msg.points.clear();
            CMNode.Topics.Pub.Lidar_VLP_front.Msg.channels.clear();
            channels.values.clear();
	
	
            //Lidar Quantity processing
            if (SimCore.State == SCState_Simulate) {
                for (int i = 0; i < LidarRSI[2].nScanPoints; i++) {
	
                    const int beam_id = LidarRSI[2].ScanPoint[i].BeamID;
                    const double azimuth = angles::from_degrees(CMNode.LidarRSI_VLP_FRONT.BeamTable[4*CMNode.LidarRSI_VLP_FRONT.rows + beam_id]);
                    const double elevation = angles::from_degrees(CMNode.LidarRSI_VLP_FRONT.BeamTable[5*CMNode.LidarRSI_VLP_FRONT.rows + beam_id]);
                    const double ray_length = 0.5 * LidarRSI[2].ScanPoint[i].LengthOF; // length of flight is back and forth
	
                    //XYZ-coordinates of scan point
                    points.x = ray_length * cos(elevation) * cos(azimuth);
                    points.y = ray_length * cos(elevation) * sin(azimuth);
                    points.z = ray_length * sin(elevation);
	
                    CMNode.Topics.Pub.Lidar_VLP_front.Msg.points.push_back(points);
                    channels.values.push_back(LidarRSI[2].ScanPoint[i].Intensity);
	
                }
                CMNode.Topics.Pub.Lidar_VLP_front.Msg.channels.push_back(channels);
                CMNode.Topics.Pub.Lidar_VLP_front.Msg.header.frame_id = CMNode.TF.Lidar_VLP_front.child_frame_id;
                CMNode.Topics.Pub.Lidar_VLP_front.Msg.header.stamp = ros::Time(LidarRSI[2].ScanTime);
            }
        }
		
		// Odometry
        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.Odometry.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.Odometry.Job), CMCRJob_RVStr(rv));
        } else if (Lidar_CycleCount % (int)CMNode.Topics.Pub.Odometry.CycleTime == 0){
            tf2::Quaternion rotation;
            CMNode.Topics.Pub.Odometry.Msg.header.frame_id = "Fr0";
            CMNode.Topics.Pub.Odometry.Msg.header.stamp = ros::Time(SimCore.Time);
            //CMNode.Topics.Pub.Odometry.Msg.pose.covariance =
            //CMNode.Topics.Pub.Odometry.Msg.twist.covariance =
            CMNode.Topics.Pub.Odometry.Msg.pose.pose.position.x = InertialSensor[0].Pos_0[0];
            CMNode.Topics.Pub.Odometry.Msg.pose.pose.position.y = InertialSensor[0].Pos_0[1];
            CMNode.Topics.Pub.Odometry.Msg.pose.pose.position.z = InertialSensor[0].Pos_0[2];
		
            rotation.setRPY(Car.Roll, Car.Pitch, Car.Yaw);
            CMNode.Topics.Pub.Odometry.Msg.pose.pose.orientation = tf2::toMsg(rotation);
		
            CMNode.Topics.Pub.Odometry.Msg.child_frame_id = "Fr1A";
            CMNode.Topics.Pub.Odometry.Msg.twist.twist.linear.x = InertialSensor[0].Vel_0[0];
            CMNode.Topics.Pub.Odometry.Msg.twist.twist.linear.x = InertialSensor[0].Vel_0[1];
            CMNode.Topics.Pub.Odometry.Msg.twist.twist.linear.z = InertialSensor[0].Vel_0[2];
            CMNode.Topics.Pub.Odometry.Msg.twist.twist.angular.x = InertialSensor[0].Omega_0[0];
            CMNode.Topics.Pub.Odometry.Msg.twist.twist.angular.x = InertialSensor[0].Omega_0[1];
            CMNode.Topics.Pub.Odometry.Msg.twist.twist.angular.z = InertialSensor[0].Omega_0[2];
        }

        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.GPS_Out.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
			LogErrF(EC_Sim, "cycleTime: %d, cycleoffset: %d, cycle: %lu", CMNode.Topics.Pub.GPS_Out.CycleTime, CMNode.Topics.Pub.GPS_Out.CycleOffset, CMNode.CycleNoRel);
			LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.GPS_Out.Job), CMCRJob_RVStr(rv));
		} else if (Lidar_CycleCount % (int)CMNode.Topics.Pub.GPS_Out.CycleTime == 0) {
			CMNode.Topics.Pub.GPS_Out.Msg.cycleno      = CMNode.CycleNoRel;
			CMNode.Topics.Pub.GPS_Out.Msg.time         = ros::Time(SimCore.Time);
			CMNode.Topics.Pub.GPS_Out.Msg.synthdelay   = CMNode.Sync.SynthDelay;
			
			ros::WallTime wtime = ros::WallTime::now();
			CMNode.Topics.Pub.GPS_Out.Msg.header.stamp.sec  = wtime.sec;
			CMNode.Topics.Pub.GPS_Out.Msg.header.stamp.nsec = wtime.nsec;
			
			unsigned int noise_deg = 150;
			double pi = 3.1415926536897932384626433832795028841971;
			CMNode.Topics.Pub.GPS_Out.Msg.latitude          = GNavSensor.Receiver.UserPosLlhTsa[0] * 180 / pi + ((double)(rand() % (noise_deg * 2 + 1)) - noise_deg) / 10000000;
			CMNode.Topics.Pub.GPS_Out.Msg.longitude         = GNavSensor.Receiver.UserPosLlhTsa[1] * 180 / pi + ((double)(rand() % (noise_deg * 2 + 1)) - noise_deg) / 10000000;
			CMNode.Topics.Pub.GPS_Out.Msg.altitude          = GNavSensor.Receiver.UserPosLlhTsa[2];
		
		}

        // write ros_imu msg 
        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.Imu.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s",CMCRJob_GetName(CMNode.Topics.Pub.Imu.Job), CMCRJob_RVStr(rv));
        } else if (Lidar_CycleCount % (int)CMNode.Topics.Pub.Imu.CycleTime == 0) {    
                        //clearing vector data to avoid overflows
            //CMNode.Topics.Pub.Imu.Msg.clear();
            tf2::Quaternion rotation;
            CMNode.Topics.Pub.Imu.Msg.header.frame_id = "Fr1A";
            CMNode.Topics.Pub.Imu.Msg.header.stamp = ros::Time(SimCore.Time);
            rotation.setRPY(Car.Roll, Car.Pitch, Car.Yaw);
            CMNode.Topics.Pub.Imu.Msg.orientation = tf2::toMsg(rotation);
            CMNode.Topics.Pub.Imu.Msg.angular_velocity.x = InertialSensor[0].Omega_B[0];
            CMNode.Topics.Pub.Imu.Msg.angular_velocity.y = InertialSensor[0].Omega_B[1];
            CMNode.Topics.Pub.Imu.Msg.angular_velocity.z = InertialSensor[0].Omega_B[2];
            
            CMNode.Topics.Pub.Imu.Msg.linear_acceleration.x = InertialSensor[0].Acc_B[0];
            CMNode.Topics.Pub.Imu.Msg.linear_acceleration.y = InertialSensor[0].Acc_B[1];
            CMNode.Topics.Pub.Imu.Msg.linear_acceleration.z = InertialSensor[0].Acc_B[2];

        }

        // write imu_senser_udp msg 
        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.imu_UDP.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s",CMCRJob_GetName(CMNode.Topics.Pub.imu_UDP.Job), CMCRJob_RVStr(rv));
        } else if (Lidar_CycleCount % (int)CMNode.Topics.Pub.imu_UDP.CycleTime == 0) { 
            tf2::Quaternion rotation; 

            CMNode.Topics.Pub.imu_UDP.Msg.Acc_0_x   = InertialSensor[0].Acc_0[0];;
            CMNode.Topics.Pub.imu_UDP.Msg.Acc_0_y   = InertialSensor[0].Acc_0[1];
            CMNode.Topics.Pub.imu_UDP.Msg.Acc_0_z   = InertialSensor[0].Acc_0[2];
            CMNode.Topics.Pub.imu_UDP.Msg.Alpha_0_x = InertialSensor[0].Alpha_0[0];
            CMNode.Topics.Pub.imu_UDP.Msg.Alpha_0_y = InertialSensor[0].Alpha_0[1];
            CMNode.Topics.Pub.imu_UDP.Msg.Alpha_0_z = InertialSensor[0].Alpha_0[2];
            CMNode.Topics.Pub.imu_UDP.Msg.Omega_0_x = InertialSensor[0].Omega_0[0];
            CMNode.Topics.Pub.imu_UDP.Msg.Omega_0_y = InertialSensor[0].Omega_0[1];
            CMNode.Topics.Pub.imu_UDP.Msg.Omega_0_z = InertialSensor[0].Omega_0[2];
            CMNode.Topics.Pub.imu_UDP.Msg.Pos_0_x   = InertialSensor[0].Pos_0[0];
            CMNode.Topics.Pub.imu_UDP.Msg.Pos_0_y   = InertialSensor[0].Pos_0[1];
            CMNode.Topics.Pub.imu_UDP.Msg.Pos_0_z   = InertialSensor[0].Pos_0[2];
            CMNode.Topics.Pub.imu_UDP.Msg.Vel_0_x   = InertialSensor[0].Vel_0[0];
            CMNode.Topics.Pub.imu_UDP.Msg.Vel_0_y   = InertialSensor[0].Vel_0[1];
            CMNode.Topics.Pub.imu_UDP.Msg.Vel_0_z   = InertialSensor[0].Vel_0[2];
            CMNode.Topics.Pub.imu_UDP.Msg.Acc_B_x   = InertialSensor[0].Acc_B[0];
            CMNode.Topics.Pub.imu_UDP.Msg.Acc_B_y   = InertialSensor[0].Acc_B[1];
            CMNode.Topics.Pub.imu_UDP.Msg.Acc_B_z   = InertialSensor[0].Acc_B[2];
            CMNode.Topics.Pub.imu_UDP.Msg.Alpha_B_x = InertialSensor[0].Alpha_B[0];
            CMNode.Topics.Pub.imu_UDP.Msg.Alpha_B_y = InertialSensor[0].Alpha_B[1];
            CMNode.Topics.Pub.imu_UDP.Msg.Alpha_B_z = InertialSensor[0].Alpha_B[2];
            CMNode.Topics.Pub.imu_UDP.Msg.Omega_B_x = InertialSensor[0].Omega_B[0];
            CMNode.Topics.Pub.imu_UDP.Msg.Omega_B_y = InertialSensor[0].Omega_B[1];
            CMNode.Topics.Pub.imu_UDP.Msg.Omega_B_z = InertialSensor[0].Omega_B[2];
            CMNode.Topics.Pub.imu_UDP.Msg.Vel_B_x   = InertialSensor[0].Vel_B[0];
            CMNode.Topics.Pub.imu_UDP.Msg.Vel_B_y   = InertialSensor[0].Vel_B[1];
            CMNode.Topics.Pub.imu_UDP.Msg.Vel_B_z   = InertialSensor[0].Vel_B[2];

        }	




        // write udp msg
        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.UDP.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s",CMCRJob_GetName(CMNode.Topics.Pub.UDP.Job), CMCRJob_RVStr(rv));
        } else if (Lidar_CycleCount % (int)CMNode.Topics.Pub.UDP.CycleTime == 0) { 
            tf2::Quaternion rotation; 

            CMNode.Topics.Pub.UDP.Msg.roll = Car.Roll;
            CMNode.Topics.Pub.UDP.Msg.pitch = Car.Pitch;
            CMNode.Topics.Pub.UDP.Msg.yaw = Car.Yaw;
            rotation.setRPY(Car.Roll, Car.Pitch, Car.Yaw);
            CMNode.Topics.Pub.UDP.Msg.orientation = tf2::toMsg(rotation);  
            CMNode.Topics.Pub.UDP.Msg.vx = Car.ConBdy1.v_1[0];
            CMNode.Topics.Pub.UDP.Msg.vy = Car.ConBdy1.v_1[1];
            CMNode.Topics.Pub.UDP.Msg.vz = Car.ConBdy1.v_1[2];
            CMNode.Topics.Pub.UDP.Msg.roll_vel = Car.RollVel;
            CMNode.Topics.Pub.UDP.Msg.pitch_vel = Car.PitchVel;
            CMNode.Topics.Pub.UDP.Msg.yaw_vel = Car.YawRate;

            CMNode.Topics.Pub.UDP.Msg.ax = Car.ConBdy1.a_1[0];
            CMNode.Topics.Pub.UDP.Msg.ay = Car.ConBdy1.a_1[1];
            CMNode.Topics.Pub.UDP.Msg.az = Car.ConBdy1.a_1[2];
            CMNode.Topics.Pub.UDP.Msg.roll_acc = Car.RollAcc;
            CMNode.Topics.Pub.UDP.Msg.pitch_acc = Car.PitchAcc;
            CMNode.Topics.Pub.UDP.Msg.yaw_acc = Car.YawAcc;


            CMNode.Topics.Pub.UDP.Msg.steer_whl_ang = Steering.IF.Ang;
            CMNode.Topics.Pub.UDP.Msg.vc_gas = VehicleControl.Gas;
            CMNode.Topics.Pub.UDP.Msg.vc_brake = VehicleControl.Brake;
            CMNode.Topics.Pub.UDP.Msg.vc_selector_ctrl = VehicleControl.SelectorCtrl;

        }		

		
		Lidar_CycleCount++;

		
	}

	UDP_Input.DriveCont.SteeringWheel = CMNode.Topics.Sub.sub_udp.Msg.SteeringWheel;
    UDP_Input.DriveCont.Ax = CMNode.Topics.Sub.sub_udp.Msg.Ax;
	UDP_Input.DriveCont.GearNo = CMNode.Topics.Sub.sub_udp.Msg.GearNo;
    UDP_PC.VC_SwitchOn = CMNode.Topics.Sub.sub_udp.Msg.VC_SwitchOn;
    
    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called close to end of CarMaker cycle
 * - See "User.c:User_Out()"
 */
int
CMRosIF_CMNode_Out (void)
{
    ros::WallTime wtime = ros::WallTime::now();

    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
	    || SimCore.State != SCState_Simulate)
	return 0;

    int rv;
    auto out = &CMNode.Topics.Pub.CM2Ext;

    /* Communicate to External ROS Node in this cycle?
     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
     */
    if ((rv = CMCRJob_DoJob(out->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing
	    && rv != CMCRJob_RV_DoSomething) {
	LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {

	out->Msg.cycleno      = CMNode.CycleNoRel;
	out->Msg.time         = ros::Time(SimCore.Time);
	out->Msg.synthdelay   = CMNode.Sync.SynthDelay;

	/* Header stamp and frame needs to be set manually! */

	/* provide system time close to data is sent */
	wtime = ros::WallTime::now();
	out->Msg.header.stamp.sec  = wtime.sec;
	out->Msg.header.stamp.nsec = wtime.nsec;


    out->Pub.publish(out->Msg);

        /* Remember cycle for debugging */
	CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }
	//Send out ros_Imu	
    auto out_imu =  &CMNode.Topics.Pub.Imu;

    if ((rv = CMCRJob_DoJob(out_imu->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing
            && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_imu->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {    

	out_imu->Pub.publish(out_imu->Msg);
    	/* Remember cycle for debugging */
	CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }


	//Send out car_UDP
    auto out_udp =  &CMNode.Topics.Pub.UDP;
    if ((rv = CMCRJob_DoJob(out_udp->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing
        && rv != CMCRJob_RV_DoSomething) {
    LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_udp->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {    


    out_udp->Pub.publish(out_udp->Msg);
    	/* Remember cycle for debugging */
    CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }
	//Send out imu_senser_udp
    auto out_imu_udp =  &CMNode.Topics.Pub.imu_UDP;
    if ((rv = CMCRJob_DoJob(out_imu_udp->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing
        && rv != CMCRJob_RV_DoSomething) {
    LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_imu_udp->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {    


    out_imu_udp->Pub.publish(out_imu_udp->Msg);
    	/* Remember cycle for debugging */
    CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }
	auto out_Lidar_VLP_left = &CMNode.Topics.Pub.Lidar_VLP_left;
	
    /* Communicate to External ROS Node in this cycle?
     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
     */
    if ((rv = CMCRJob_DoJob(out_Lidar_VLP_left->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing
            && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_Lidar_VLP_left->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {
	
	out_Lidar_VLP_left->Pub.publish(out_Lidar_VLP_left->Msg);
	
	/* Remember cycle for debugging */
	CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }

	
	auto out_Lidar_VLP_right = &CMNode.Topics.Pub.Lidar_VLP_right;
	
    /* Communicate to External ROS Node in this cycle?
     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
     */
    if ((rv = CMCRJob_DoJob(out_Lidar_VLP_right->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing
            && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_Lidar_VLP_right->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {
	
	out_Lidar_VLP_right->Pub.publish(out_Lidar_VLP_right->Msg);
	
	/* Remember cycle for debugging */
	CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }

	auto out_Lidar_VLP_front = &CMNode.Topics.Pub.Lidar_VLP_front;
	
    /* Communicate to External ROS Node in this cycle?
     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
     */
    if ((rv = CMCRJob_DoJob(out_Lidar_VLP_front->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing
            && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_Lidar_VLP_front->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {
	
	out_Lidar_VLP_front->Pub.publish(out_Lidar_VLP_front->Msg);
	
	/* Remember cycle for debugging */
	CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }

    auto out_Odometry = &CMNode.Topics.Pub.Odometry;
	
    if ((rv = CMCRJob_DoJob(out_Odometry->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing
            && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_Odometry->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {
	
        geometry_msgs::TransformStamped tf;
        tf.transform.rotation = CMNode.Topics.Pub.Odometry.Msg.pose.pose.orientation;
        tf.transform.translation.x = CMNode.Topics.Pub.Odometry.Msg.pose.pose.position.x;
        tf.transform.translation.y = CMNode.Topics.Pub.Odometry.Msg.pose.pose.position.y;
        tf.transform.translation.z = CMNode.Topics.Pub.Odometry.Msg.pose.pose.position.z;
        tf.header = CMNode.Topics.Pub.Odometry.Msg.header;
        tf.child_frame_id = CMNode.Topics.Pub.Odometry.Msg.child_frame_id;
        CMNode.TF.br->sendTransform(tf);
	
        out_Odometry->Pub.publish(out_Odometry->Msg);
    }


	auto out_gps_out = &CMNode.Topics.Pub.GPS_Out;
	
    /* Communicate to External ROS Node in this cycle?
     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
     */
    if ((rv = CMCRJob_DoJob(out_gps_out->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing
            && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_gps_out->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {
	
	out_gps_out->Pub.publish(out_gps_out->Msg);
	
	/* Remember cycle for debugging */
	CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }
	
    /* Publish "/clock" topic after all other other topics are published
     * - Is the order of arrival in other node identical? */
    if (CMNode.Cfg.nCyclesClock > 0 && CMNode.CycleNoRel%CMNode.Cfg.nCyclesClock == 0) {
	CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(SimCore.Time);
	CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
    }
	
    /* ToDo: When increase? */
    CMNode.CycleNoRel++;

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called one Time when CarMaker ends
 * - See "User.c:User_End()"
 */
int
CMRosIF_CMNode_End (void)
{
	CMNode.Topics.Pub.Lidar_VLP_left.Msg.points.clear();
    CMNode.Topics.Pub.Lidar_VLP_left.Msg.channels.clear();
	
	CMNode.Topics.Pub.Lidar_VLP_right.Msg.points.clear();
    CMNode.Topics.Pub.Lidar_VLP_right.Msg.channels.clear();

	CMNode.Topics.Pub.Lidar_VLP_front.Msg.points.clear();
    CMNode.Topics.Pub.Lidar_VLP_front.Msg.channels.clear();
	
	free(CMNode.LidarRSI_VLP_LEFT.BeamTable);
	free(CMNode.LidarRSI_VLP_RIGHT.BeamTable);
	free(CMNode.LidarRSI_VLP_FRONT.BeamTable);


    LOG("%s: End", __func__);

    if (ros::isInitialized()) {

	/* Needs to be called before program exists, otherwise
	 * "boost" error due to shared library and default deconstructor */
	CMNode.Cfg.Ros.Node->shutdown();

	/* ToDo:
	 * - Blocking call? Wait until shutdown has finished?
	 * - Optional? */
	ros::shutdown();
    }

    return 1;
}



/*!
 * Important:
 * - NOT automatically called by CMRosIF extension
 *
 * Description:
 * - Example of user generated function
 * - Can be accessed in other sources, e.g. User.c
 * - Use "CMRosIF_GetSymbol()" to get symbol (see "lib/CMRosIF.h")
 *
 */
int
CMRosIF_CMNode_MyFunc (char *LogMsg)
{

    LOG("%s: %s",  __func__, LogMsg);
    return 1;
}

#ifdef __cplusplus
}
#endif

