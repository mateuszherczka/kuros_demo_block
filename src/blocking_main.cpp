#include <iostream>
#include <boost/format.hpp>

#include <kuros.h>
#include <HandlingServer.hpp>
#include <DataFile.hpp>


int main()

{
    // load a space-delimited file
    DataFile dataFile;
    std::string filename = "trajectory.txt";

    //vector of frames std::vector< frame_vec >
    trajectory_vec trajectory;  // trajectory vector

    dataFile.loadSDFrames(filename, trajectory);  // store file contents in trajectory vector

    if (trajectory.empty())
    {
        cerr << "Failed loading trajectory, bailing out!" << endl;
        return 0;
    }

    // set parameters for trajectory
    info_vec info(KUKA_INFO_SIZE,1);  // make an info vector, init all values to 1

    info[KUKA_RMODE]        = 1;    // response mode
    info[KUKA_RMS]          = 100;  // [ms] response stream interval
    info[KUKA_TRAJID]       = 666;  // a trajectory id
    info[KUKA_RUN]          = 1;    // 0 == robot exits after trajectory, 1 == robot keeps running
    info[KUKA_VEL]          = 200;  // [mm/sec], comfortable vel = 200
    info[KUKA_TOL]          = 20;   // [mm], max approximation distance from trajectory point
    info[KUKA_FRAMETYPE]    = 1;    // 1 == cartesian X Y Z Y P R, 2 == joint A1 A2 A3 A4 A5 A6 (not yet supported)

    cout << "Have " << trajectory.size() << " frames to send, starting server." << endl;

    // this time HandlingServer inherits from BlockingServer, look in class definition
    HandlingServer aserver;     // loads config file on creation

    aserver.startListening();   // blocks until connection

    cout << "Waiting a little." << endl;
    boost::this_thread::sleep( boost::posix_time::milliseconds(1000));

    cout << "Sending (blocking) trajectory with " << trajectory.size() << " frames." << endl;

    // send trajectory previously loaded from file
    aserver.blockSendTrajectory(info, trajectory);

    cout << "Done, continue." << endl;

    // let's make a trajectory with first, last and some other points
    // look how the robot interpolates them
    trajectory_vec pointSampleTrajectory;
    pointSampleTrajectory.push_back(trajectory.front());
    pointSampleTrajectory.push_back(trajectory[10]);
    pointSampleTrajectory.push_back(trajectory[20]);
    pointSampleTrajectory.push_back(trajectory[30]);
    pointSampleTrajectory.push_back(trajectory.back());

    cout << "Sending (blocking) trajectory with " << pointSampleTrajectory.size() << " frames." << endl;

    // send it, reusing the info vector we already made
    info[KUKA_TRAJID] = 667;
    aserver.blockSendTrajectory(info, pointSampleTrajectory);

    cout << "Done, continue." << endl;

    // lets do the same but send each frame as a separate pose
    // these points won't be interpolated
    trajectory_vec pose;

    cout << "Sending (blocking) pose." << endl;

    pose.push_back(trajectory.front());
    info[KUKA_TRAJID] = 668;
    aserver.blockSendTrajectory(info, pose);

    cout << "Done, Sending (blocking) next pose." << endl;

    pose[0] = trajectory[10];
    info[KUKA_TRAJID] = 669;
    aserver.blockSendTrajectory(info, pose);

    cout << "Done, Sending (blocking) next pose." << endl;

    pose[0] = trajectory[20];
    info[KUKA_TRAJID] = 670;
    aserver.blockSendTrajectory(info, pose);

    cout << "Done, Sending (blocking) next pose." << endl;

    pose[0] = trajectory[30];
    info[KUKA_TRAJID] = 671;
    aserver.blockSendTrajectory(info, pose);

    cout << "Done, Sending (blocking) next pose." << endl;

    pose[0] = trajectory.back();
    info[KUKA_TRAJID] = 672;
    info[KUKA_RUN] = 0; // for the last trajectory we send Run = 0, robot will exit
    aserver.blockSendTrajectory(info, pose);

    cout << "Done." << endl;


    cout << "Idle, waiting for robot to terminate connection." << endl;

    // idle until
    while( aserver.isConnected() ) {

        boost::this_thread::sleep( boost::posix_time::milliseconds(100));   // idle
    }

    // we sent Run == 0 so robot exits when done, server disconencts and we quit.
    cout << "Done, exiting." << endl;
    return 0;
}
