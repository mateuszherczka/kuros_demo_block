#include <iostream>
#include <boost/format.hpp>

#include <kuros.h>
#include <HandlingServer.hpp>
#include <DataFile.hpp>


int main()

{
    // trajectory vectors we will use
    trajectory_vec trajectory;
    trajectory_vec pointSampleTrajectory;
    trajectory_vec pose;

    // info vector, init all values to some reasonable number
    info_vec info(KUKA_INFO_SIZE,1);

    // the trajectory file
    std::string filename = "trajectory.txt";

    // load a space-delimited file
    DataFile dataFile;
    dataFile.loadSDFrames(filename, trajectory);  // store file contents in trajectory vector

    if (trajectory.empty())
    {
        cerr << "Failed loading trajectory, bailing out!" << endl;
        return 0;
    }

    cout << "Loaded " << trajectory.size() << " frames, starting server." << endl;

    // this time HandlingServer inherits from BlockingServer, look in class definition
    HandlingServer aserver;
    aserver.startListening();   // blocks until connection

    // set parameters for trajectory
    info[KUKA_RMODE]        = KUKA_RMODE_BASIC;     // response mode
    info[KUKA_RMS]          = 100;                  // [ms] response stream interval
    info[KUKA_TRAJID]       = 666;                  // a trajectory id
    info[KUKA_RUN]          = YES;                  // NO robot exits after trajectory, YES robot keeps running
    info[KUKA_VEL]          = 200;                  // [mm/sec], comfortable vel = 200
    info[KUKA_TOL]          = 20;                   // [mm], max approximation distance from trajectory point
    info[KUKA_FRAMETYPE]    = KUKA_CARTESIAN;       // KUKA_CARTESIAN X Y Z Y P R, KUKA_AXIS A1 A2 A3 A4 A5 A6 (not yet supported)

    // sample points for second trajectory
    pointSampleTrajectory.push_back(trajectory.front());
    pointSampleTrajectory.push_back(trajectory[10]);
    pointSampleTrajectory.push_back(trajectory[20]);
    pointSampleTrajectory.push_back(trajectory[30]);
    pointSampleTrajectory.push_back(trajectory.back());


    cout << "Waiting a little." << endl;
    boost::this_thread::sleep( boost::posix_time::milliseconds(1000));

    // Lets keep sending trajectories over and over until user quits
    while (aserver.isConnected())
    {

        cout << "Sending (blocking) trajectory with " << trajectory.size() << " frames." << endl;
        aserver.blockSendTrajectory(info, trajectory);  // blockSendTrajectory() blocks

        cout << "Done, continue." << endl;

        cout << "Sending (blocking) sampled trajectory with " << pointSampleTrajectory.size() << " frames." << endl;
        ++info[KUKA_TRAJID];
        aserver.blockSendTrajectory(info, pointSampleTrajectory);

        cout << "Done, continue." << endl;

        cout << "Sending (blocking) pose." << endl;

        ++info[KUKA_TRAJID];
        pose.push_back(trajectory.front());
        aserver.blockSendTrajectory(info, pose);

        cout << "Done, Sending (blocking) next pose." << endl;

        ++info[KUKA_TRAJID];
        pose[0] = trajectory[10];
        aserver.blockSendTrajectory(info, pose);

        cout << "Done, Sending (blocking) next pose." << endl;

        ++info[KUKA_TRAJID];
        pose[0] = trajectory[20];
        aserver.blockSendTrajectory(info, pose);

        cout << "Done, Sending (blocking) next pose." << endl;

        ++info[KUKA_TRAJID];
        pose[0] = trajectory[30];
        aserver.blockSendTrajectory(info, pose);

        cout << "Done, Sending (blocking) next pose." << endl;

        ++info[KUKA_TRAJID];
        pose[0] = trajectory.back();
        aserver.blockSendTrajectory(info, pose);

        // since we loop, clear the pose vector
        pose.clear();

        cout << "Done. Looping." << endl;
    }

    return 0;
}
