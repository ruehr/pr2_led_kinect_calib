#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>


#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <ethercat_trigger_controllers/SetWaveform.h>

#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

#include <pcl/registration/transformation_estimation_svd.h>


std::string mount_frame_ = "head_mount_link";
std::string rgb_optical_frame_ = "head_mount_kinect_ir_link";
std::string rgb_topic_ = "/head_mount_kinect/depth_registered/points";
std::string led_frame_ = "r_gripper_led_frame";

tf::TransformListener*listener_ = 0L;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

PointHeadClient* point_head_client_;

void init()
{
    if (!listener_)
        listener_ = new tf::TransformListener();
}

tf::Stamped<tf::Pose> getPose(const char target_frame[],const char lookup_frame[], ros::Time tm = ros::Time(0))
{

    init();
    ros::Rate rate(100.0);

    tf::StampedTransform transform;
    bool transformOk = false;
    bool had_to_retry = false;

    //listener_->waitForTransform(target_frame, lookup_frame, tm, ros::Duration(0.5));
    while (ros::ok() && (!transformOk))
    {
        transformOk = true;
        try
        {
            listener_->lookupTransform(target_frame, lookup_frame,tm, transform);
        }
        catch (tf::TransformException ex)
        {
            std::string what = ex.what();
            what.resize(100);
            ROS_INFO("getPose: tf::TransformException ex.what()='%s', will retry",what.c_str());
            transformOk = false;
            had_to_retry = true;
            //listener_->waitForTransform(target_frame, lookup_frame, ros::Time(0), ros::Duration(0.5));
        }
        if (!transformOk)
            rate.sleep();
    }

    if (had_to_retry)
        ROS_INFO("Retry sucessful");

    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());

    return ret;
}

void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string frame_id, ros::Time after, ros::Time *tm)
{

    sensor_msgs::PointCloud2 pc;
    bool found = false;
    while (!found)
    {
        pc  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(rgb_topic_));
        if ((after == ros::Time(0,0)) || (pc.header.stamp > after))
            found = true;
        else
        {
            //ROS_ERROR("getKinectCloudXYZ cloud too old : stamp %f , target time %f",pc.header.stamp.toSec(), after.toSec());
        }
    }
    if (tm)
        *tm = pc.header.stamp;

    tf::Stamped<tf::Pose> net_stamped = getPose(rgb_optical_frame_.c_str(),pc.header.frame_id.c_str());
    tf::Transform net_transform;
    net_transform.setOrigin(net_stamped.getOrigin());
    net_transform.setRotation(net_stamped.getRotation());

    sensor_msgs::PointCloud2 pct; //in map frame

    pcl_ros::transformPointCloud(frame_id.c_str(),net_transform,pc,pct);
    pct.header.frame_id = frame_id.c_str();

    geometry_msgs::Transform t_msg;
    tf::transformTFToMsg(net_transform, t_msg);

    //std::cout << "CLOUD transf" << pc.header.frame_id << " to " << pct.header.frame_id << " : " << t_msg << std::endl;

    pcl::fromROSMsg(pct, *cloud);
}


void lookAt(std::string frame_id, double x, double y, double z)
{
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x;
    point.point.y = y;
    point.point.z = z;
    goal.target = point;

    //we are pointing the wide_stereo camera frame
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = rgb_optical_frame_;

    //take at least 5 seconds to get there
    goal.min_duration = ros::Duration(2);

    //and go no faster than 0.1 rad/s
    goal.max_velocity = 0.5;

    //send the goal
    point_head_client_->sendGoal(goal);

    point_head_client_->waitForResult();
    //std::cout << "lookat finished" << std::endl;
}

ros::ServiceClient client;
ethercat_trigger_controllers::SetWaveform waveform;

std::vector<double> field;

tf::Vector3 locate_led()
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    bool have_last_cloud = false;

    field.resize(640*480*2);
    for (std::vector<double>::iterator it= field.begin(); it!= field.end(); ++it)
        *it = 0;

    double field_max = -1000;

    tf::Vector3 best_pt;

    double field_avg = 0;

    // trigger a few times until we have enough evidence we found the led
    while (ros::ok() && ((field_max<2500) || (field_max < fabs(field_avg) * 100)))
    {
        if (waveform.request.active_low)
            waveform.request.active_low = 0;
        else
            waveform.request.active_low = 1;

        double sign = waveform.request.active_low == 1 ? 1 : -1;

        if (client.call(waveform))
        {
            //ROS_INFO("CALLED %f ", sign);
        }

        ros::Duration(0.01).sleep(); // led should be switched instantly but lets be safe and give it some time to 'settle'
        ros::Time lookup_time;

        *last_cloud = *cloud;

        getCloud(cloud, rgb_optical_frame_, ros::Time::now(), &lookup_time);

        size_t max_index = 0;
        field_avg = 0;

        if (have_last_cloud) {
            for (size_t k = 0; k < std::min(last_cloud->points.size(), cloud->points.size()); k++)
            {
                field[k] += ((cloud->points[k].r - last_cloud->points[k].r) +
                             (cloud->points[k].g - last_cloud->points[k].g) +
                             (cloud->points[k].b - last_cloud->points[k].b))  * sign;
                field_avg += fabs(field[k]);
                if (field[k]>field_max) {
                    field_max = field[k];
                    max_index = k;
                }
            }
            field_avg /= cloud->points.size();
            tf::Vector3 max_pt(cloud->points[max_index].x, cloud->points[max_index].y, cloud->points[max_index].z);
            best_pt = max_pt;
            std::cout << "max pt idx" << max_index << " (" << field_max << "):" << "field abs avg " << field_avg << std::endl;
            //max_pt.x() << " " << max_pt.y() << " " << max_pt.z() << std::endl;
            //std::cout << "field avg " << field_avg << std::endl;
        } else {
            have_last_cloud = true;
        }

        //rt.sleep();
    }
    return best_pt;
}

void find_correspondance(tf::Vector3 &led_k, tf::Vector3 &led_tf)
{
        led_k = locate_led();
        tf::Stamped<tf::Pose> led_tf_pose = getPose(rgb_optical_frame_.c_str(),led_frame_.c_str());
        led_tf = led_tf_pose.getOrigin();
        if (!ros::ok())
            exit(0);

        std::cout << "Found point correspondance: \n" <<
                     "in kinect [ " << led_k.x() << " ," << led_k.y() << " , " << led_k.z() << " ] \n" <<
                     "in tf     [ " << led_tf.x() << " ," << led_tf.y() << " , " << led_tf.z() << " ] \n" << std::endl;
}

int main(int argc,char **argv)
{

    ros::init(argc, argv, "led_calibration_executive");
    ros::NodeHandle nh;

    init();

    client = nh.serviceClient<ethercat_trigger_controllers::SetWaveform>("/r_gripper_led/set_waveform");
    //ethercat_trigger_controllers::SetWaveform waveform;
    waveform.request.rep_rate = 1;
    waveform.request.phase = 0;
    waveform.request.duty_cycle = 0.5;
    waveform.request.running = 0;
    waveform.request.active_low = 0;
    waveform.request.pulsed = 0;

    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

    while (!point_head_client_->waitForServer(ros::Duration(5.0)) && ros::ok())
    {
        ROS_INFO("Waiting for the point_head_action server to come up");
    }


    ros::Rate rt(2);

    std::vector<double> field;
    field.resize(640*480*2);
    for (std::vector<double>::iterator it= field.begin(); it!= field.end(); ++it)
        *it = 0;

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimator;
    pcl::PointCloud<pcl::PointXYZ> points_in_kinect;
    pcl::PointCloud<pcl::PointXYZ> points_in_tf;

    double angle = 0;
    while (ros::ok())
    {
        angle += 37.0 / 180.0 * M_PI;
        double radius = .2;
        lookAt(led_frame_.c_str(), radius * sin(angle), radius * cos(angle),0);
        //std::cout << angle << " " << radius * sin(angle) << " " <<  radius * cos(angle) << std::endl;

        // wait for the head turning to finish
        //  ros::Duration(5).sleep();
        ros::Rate rt(10);
        ros::Time start = ros::Time::now();
        tf::Stamped<tf::Pose> hand_pose = getPose(rgb_optical_frame_.c_str(),led_frame_.c_str());
        tf::Stamped<tf::Pose> last_hand_pose;
        double position_change = 5;
        size_t num_settled = 0;
        // wait for head vs hand tf to settle as the head should not move during calib
        while  (num_settled < 3) {
            rt.sleep();
            last_hand_pose = hand_pose;
            hand_pose = getPose(rgb_optical_frame_.c_str(),led_frame_.c_str());
            //std::cout << "time " << (ros::Time::now() - start).toSec() << " pos " << (hand_pose.getOrigin() - last_hand_pose.getOrigin()).length() <<
              //  " angle " << hand_pose.getRotation().angle(last_hand_pose.getRotation()) << std::endl;
            position_change = (hand_pose.getOrigin() - last_hand_pose.getOrigin()).length();
            if (position_change < 0.0001)
                num_settled++;
            else
                num_settled = 0;
        }

        tf::Vector3 led_tf, led_kinect;
        find_correspondance(led_kinect, led_tf);
        pcl::PointXYZ pt_k(led_kinect.x(),led_kinect.y(),led_kinect.z());
        pcl::PointXYZ pt_tf(led_tf.x(),led_tf.y(),led_tf.z());

        if (led_kinect.x() != led_kinect.x()) {
            ROS_WARN("Last point had nan position, skipping it");
        } else {
            points_in_kinect.points.push_back(pt_k);
            points_in_tf.points.push_back(pt_tf);
        }

        tf::Stamped<tf::Pose> kinect_pose = getPose(mount_frame_.c_str() ,rgb_optical_frame_.c_str());
        tf::Transform kinect_trans = kinect_pose;

        if (points_in_tf.points.size() > 2) {
            Eigen::Matrix4f trans;
            transformation_estimator.estimateRigidTransformation(points_in_kinect,points_in_tf, trans);

            Eigen::Matrix4d md(trans.cast<double>());
            Eigen::Affine3d affine(md);
            tf::Transform transform;
            tf::TransformEigenToTF(affine, transform);

            geometry_msgs::Transform ps;
            tf::transformTFToMsg(transform,ps);
            std::cout << "Correction relative to current kinect pose:"<< std::endl;
            std::cout << ps;
            std::cout << "relative angle: " << transform.getRotation().getAngle() << " rad or " <<  transform.getRotation().getAngle() * 180.0f / M_PI << " degrees " << std::endl << std::endl;

            //geometry_msgs::Transform ps_kinect;
            //tf::transformTFToMsg(kinect_trans,ps_kinect);
            //std::cout << "kinect pose " << ps_kinect << std::endl;

            tf::Transform post = kinect_trans * transform;
            tf::Transform pre = transform * kinect_trans;
            geometry_msgs::Transform ps_post;
            tf::transformTFToMsg(post,ps_post);
            std::cout << "New pose of " <<  rgb_optical_frame_  << " in " << mount_frame_ << " \n " << ps_post << std::endl;

            btQuaternion q;
            double roll, pitch, yaw;
            tf::Matrix3x3(post.getRotation()).getRPY(roll, pitch, yaw);
            std::cout << "RPY " << roll << " " << pitch << " " << yaw <<  std::endl;

            //geometry_msgs::Transform ps_pre;
            //tf::transformTFToMsg(pre,ps_pre);
            //std::cout << "kinect pose pre " << ps_pre << std::endl;

            //std::cout << trans << std::endl;
        }
    }


}


//rosservice call /r_gripper_led/set_waveform '{rep_rate: 1, phase: 0, duty_cycle: 0.5, running: 0, active_low: 0, pulsed: 0}'

//rosservice call /l_gripper_led/set_waveform '{rep_rate: 1, phase: 0, duty_cycle: 0.5, running: 0, active_low: 0, pulsed: 0}'
