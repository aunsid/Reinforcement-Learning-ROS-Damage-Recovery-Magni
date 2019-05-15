# include <ros/ros.h>
# include <geometry_msgs/Twist.h>
# include <nav_msgs/Odometry.h>
# include <math.h> 
# include <fstream>
# include <type_traits>
# include <tf/transform_datatypes.h>
# define PI  3.14159265

using namespace std;
double g_x;
double g_y;
double g_z;
double z_ang;
double combinedAngle;

double publishAction(int x)
{	double rad;
	double action[]= {-50,-25,-15,-5,0,5,15,25,50};		//pos=left, neg=right
	rad = action[x]* (PI /180);
	return rad;
}

// This has been modified for x being fwd and y perpendicular to the bot 
double state (double x_w,  double y_w, double z_ang)
{
	// x_w and y_w are the position of the robot in the world frame 
	double x_pos = x_w;
	double x_end = 3.5;
	double ref_x = 0.0;
	ref_x = x_pos+0.5;

	double alpha = (-z_ang + atan2(-y_w,ref_x - x_pos))*180/PI;

	return alpha;
}

void poseMsgReceived( const  nav_msgs::Odometry  & msg){
	g_x = msg.pose.pose.position.x;
	g_y = msg.pose.pose.position.y;
	g_z = msg.pose.pose.position.z;
	z_ang = tf::getYaw(msg.pose.pose.orientation);
	
	
	double alpha = state(g_x, g_y, z_ang);
	combinedAngle = alpha;

	
}

int getState(const double inputAngle)
{	
	int stateindex;
	if( inputAngle < -20)
 		stateindex = 9;
	else if( inputAngle < -15)
 		stateindex = 8;
	else if( inputAngle < -10)
 		stateindex = 7;
	else if( inputAngle < -5)
 		stateindex = 6;
	else if( inputAngle < 0)
 		stateindex = 5;
	else if(inputAngle < 5)
 		stateindex = 4;
	else if( inputAngle < 10)
 		stateindex = 3;
	else if( inputAngle < 15 )
 		stateindex = 2;
        else if( inputAngle < 20 )
        	stateindex = 1;
        else 
        	stateindex = 0;

	return stateindex;
}

int main(int argc, char **  argv){
	// initialize the ros system and node
	ros::init (argc, argv, "subodom");
	ros::NodeHandle n;
	//subscribe to ROS nodes
	ros::Subscriber sub = n.subscribe ("/odom", 1000, &poseMsgReceived);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);// publish at a slower time
	ros::Rate rate(2);
	srand(time(NULL));

	//Initialize User variables
	int expRate = 1;		//1=explore, 0=implement
        double discount =0;  	//Higher means more discounting
	int exCtMax = 2; 	//Higher gives the robot longer to recover but also adds more potentially useless movements to qVal

	//Initialize Program Variables
	double reward= 0;
	double cumulativeReward = 0;
	int oldState=0;
	int  currentState;
	double combinedOld = 0;
	double combinedNew = 0;
        double learnRate=0.25; //Higher is faster
	int actionInt =0;
	double actionAng =0.0;
	int iterations = 0;
	int extremeStateCounter=0;
	double qVals[10][9]={	//qVals[oldState][action]
				{-100,-100,-100,-100,-100,-100,-100,-100,-100},
				{-100,-100,-100,-100,-100,-100,-100,-100,-100},
				{-100,-100,-100,-100,-100,-100,-100,-100,-100},
				{-100,-100,-100,-100,-100,-100,-100,-100,-100},
				{-100,-100,-100,-100,-100,-100,-100,-100,-100},
				{-100,-100,-100,-100,-100,-100,-100,-100,-100},
				{-100,-100,-100,-100,-100,-100,-100,-100,-100},
				{-100,-100,-100,-100,-100,-100,-100,-100,-100},
				{-100,-100,-100,-100,-100,-100,-100,-100,-100},
				{-100,-100,-100,-100,-100,-100,-100,-100,-100}
			};
	int nActions = extent<decltype(qVals),1>::value;
	double qNum[10][9]={	//qNum[oldState][action]
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0}
			};

	//Load previously saved qVals matrix
	ifstream in;
	in.open("qlearnHC_v4.txt");
	if (!in){
		
		ROS_INFO_STREAM("qVal Matrix: 0s");
	}
	else {
		// Read the saved Q matrix

		ROS_INFO_STREAM("qVal Matrix: from qlearnHC_v4.txt");
		for (int s = 0; s < 10; s++) {
			for (int a = 0; a < 9; a++) {
				in >> qVals[s][a];
			}
		}
	}
	in.close();

	//Load previously saved qNum matrix
	ifstream num;
	in.open("qNumHC_v4.txt");
	if (!num){
		
		ROS_INFO_STREAM("qNum Matrix: 0s");
	}
	else {
		// Read the saved Q matrix

		ROS_INFO_STREAM("qNum Matrix: from qlearnHC_v4.txt");
		for (int s = 0; s < 10; s++) {
			for (int a = 0; a < 9; a++) {
				in >> qNum[s][a];
			}
		}
	}
	in.close();

	//Main loop
	while (ros::ok() && (g_x) < 4.0)
	{
		//Observe new/current state
		combinedOld = combinedNew;
		oldState = getState(combinedOld);
		combinedNew = combinedAngle;
		currentState = getState(combinedNew);
		ROS_INFO_STREAM("2. old angle:   "<<combinedOld<<"  new angle:   "<<combinedNew);
		ROS_INFO_STREAM("3. old state:   "<<oldState<<"  new state:   "<<currentState);


		//Calculate reward for previous action
		reward = -abs(4.5-currentState);
		cumulativeReward += reward;
		ROS_INFO_STREAM("4. Reward :" <<reward);
		
		//Get max qVal of current state (for new qVal calculation)
		int maxQ = -1000000;
		for (int a = 0; a <9; a++)
		{
			int val = qVals[currentState][a];
			maxQ = max(maxQ,val);
		}

		//Mode dependent stuff
		int minQnum = 1000000;
		int minQidx[] = {0,0,0,0,0,0,0,0,0};
		int minQn = 0;
		int val =0;
		if (expRate == 1){
			//Exploring
			//Update old state's qVal (motors don't move for first 3 loops)
			if (extremeStateCounter <= exCtMax && iterations >= 3) {
				//Update qVal & qNum of old state
				//learnRate = pow(0.75,qNum[oldState][actionInt]);
				double oldVal = qVals[oldState][actionInt];
				qVals[oldState][actionInt] = (1 - learnRate)*qVals[oldState][actionInt] + learnRate*(reward + discount * maxQ);
				qNum[oldState][actionInt] ++;
				ROS_INFO_STREAM("5.Saved qVal:  "<< qVals[oldState][actionInt]);
				ROS_INFO_STREAM("6.Learn Rate:  "<< learnRate);
				ROS_INFO_STREAM("7.qVal difference:  "<< qVals[oldState][actionInt]-oldVal);
				if (qVals[oldState][actionInt] == 0) {
					ROS_INFO_STREAM("QVAL ERROR:   ###########################################");
				}
				if (abs(qVals[oldState][actionInt] - oldVal) > 2*learnRate) {
					ROS_INFO_STREAM("CHANGE BIGGER THAN 2 STATES:   ###########################################");
				}
			}
			ROS_INFO_STREAM("\n");

			//Is current state extreme?
			if (currentState == 0 || currentState == 9){
				extremeStateCounter ++;		
			}
			else {
				extremeStateCounter =0;
			}
			
			//Find new action
			if (extremeStateCounter <= exCtMax) {
				//Find indices of least trained qVals
				for (int a = 0; a <9; a++)
				{
					val = qNum[currentState][a];
					if (val < minQnum) {
						minQnum = val;
						minQn = 1;
						minQidx[minQn-1] = a;
					}
					else if (val == minQnum) {
						minQn ++;
						minQidx[minQn-1] = a;
					}
				}
			
				//Choose a random one of the least known actions
				actionInt = minQidx[rand() % minQn];	
			}
			else if (currentState == 0){
				//Force correction
				actionInt = 8;
			}
			else if (currentState == 9){
				//Force correction
				actionInt = 0;
			}
		}
		else {
			//Implementing - Choose optimal action
			double maxQ = -10000000;
			int maxQidx = 4;
			for (int actionNum=0;actionNum<=8;actionNum++){
				if (qVals[currentState][actionNum] > maxQ) {
					maxQidx = actionNum;
					maxQ = qVals[currentState][actionNum];
				}
			}
			actionInt = maxQidx;
			ROS_INFO_STREAM("\n");
		}

		//Sets speed
		geometry_msgs::Twist msg;
		msg.linear.x = 0.2;

		//Apply the action with handicap
		ROS_INFO_STREAM("1. Action :" << actionInt);
		actionAng = publishAction(actionInt); // converts action# to angVel
		msg.angular.z = actionAng;  // publish angular value 
		msg.angular.z = msg.angular.z - 0.1; // added handicap, whatever the action that was choosen - 0.1 
		pub.publish(msg); // apply angular action

		//Output into to terminal
		//ROS_INFO_STREAM("combined angle:    "<<combinedNew);
		//ROS_INFO_STREAM("linear x:  "<<  g_x);
		//ROS_INFO_STREAM("linear y:  "<<  g_y);
		//ROS_INFO_STREAM("reward  "<<reward);
		//ROS_INFO_STREAM("actionAng "<<  actionAng);
		//ROS_INFO_STREAM("actionAng index "<<  actionInt);
		//ROS_INFO_STREAM("z angle:   "<<z_ang*180/PI);
		//ROS_INFO_STREAM("combined old "<<combinedOld);
		
		//Finish main loopqVal
		ros::spinOnce();
		rate.sleep();
		iterations ++;
	}

	//Save updated qVal matrix to file
	ROS_INFO_STREAM("Starting File Write");
	fstream myfile;
	myfile.open("qlearnHC_v4.txt", fstream::out);
	for (int s = 0; s< 10; s++) {
		// Prints row of Q
		for (int a = 0; a<9; a++)
		{
			//Print each value
			myfile << setprecision(4)<< qVals[s][a] << "\t";
		}
		myfile << std::endl;
	}
	myfile.close();
	ROS_INFO_STREAM("qVals complete!");

	//Save updated qNum matrix to file
	fstream myfilenum;
	myfilenum.open("qNumHC_v4.txt", fstream::out);
	for (int s = 0; s< 10; s++) {
		// Prints row of Q
		for (int a = 0; a<9; a++)
		{
			//Print each value
			myfilenum << qNum[s][a] << "\t";
		}
		myfilenum << std::endl;
	}
	myfilenum.close();
	ROS_INFO_STREAM("qNum complete!");

	//Save cumulative reward and number of iterations
	if (expRate == 0){
		myfile.open("cumulativeRewards_v4.txt", std::ios_base::app);
		myfile << "Cumulative Reward: " << cumulativeReward << "\tIterations:" << iterations << std::endl;
		myfile.close();
		ROS_INFO_STREAM("CumulativeRewards complete!");
	}

	return 0;
}
