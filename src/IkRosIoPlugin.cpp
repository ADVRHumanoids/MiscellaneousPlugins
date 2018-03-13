#include <MiscellaneousPlugins/IkRosIoPlugin.h>
#include <boost/bind.hpp>

REGISTER_XBOT_IO_PLUGIN(IkRosIo, MiscPlugins::IkRosIoPlugin)


bool MiscPlugins::IkRosIoPlugin::init(std::string path_to_config_file, XBot::SharedMemory::Ptr shmem)
{
    _topic_names.push_back("xsens_joint");
    _topic_names.push_back("xsens_stiff");
    _topic_names.push_back("xsens_damp");
    _pipe_names = _topic_names;

    _pub_nrt_joint = XBot::PublisherNRT<Eigen::Matrix<double,7,1>>(_pipe_names[0]);
    _pub_nrt_stiff = XBot::PublisherNRT<Eigen::Matrix<double,4,4>>(_pipe_names[1]);
    _pub_nrt_damp = XBot::PublisherNRT<Eigen::Matrix<double,4,4>>(_pipe_names[2]);
    
    ros::NodeHandle n;

    for(int i = 0; i < _topic_names.size(); i++){
        std::string topic_name = _topic_names[i];
        _sub.push_back( n.subscribe<std_msgs::Float64MultiArray>(topic_name,
                                                                1,
                                                                boost::bind(&MiscPlugins::IkRosIoPlugin::callback,
                                                                            this,
                                                                            _1, i)
        ) );
    }
}

void MiscPlugins::IkRosIoPlugin::run()
{
    ros::spinOnce();
}

void MiscPlugins::IkRosIoPlugin::close()
{
    return;
}


void MiscPlugins::IkRosIoPlugin::callback(std_msgs::Float64MultiArrayConstPtr msg, int id)
{
    std::vector<double> data;
    data = msg->data;    
    if(id == 0){      
      Eigen::Matrix<double,7,1> tmp;
      for(int i=0; i< data.size(); i++){
        tmp[i] = data [i];
      }
      _pub_nrt_joint.write(tmp);
    }else if(id == 1){
      Eigen::Matrix<double,4,4> tmp;
      for(int i=0, j=0,k=0; i< data.size(); i++){
        tmp(j,k) = data [i];
        k++;
        if(k==4){
          j++;
          k=0;
        }
      }
       _pub_nrt_stiff.write(tmp);
    }else if( id == 2){
      Eigen::Matrix<double,4,4> tmp;
      for(int i=0, j=0,k=0; i< data.size(); i++){
        tmp(j,k) = data [i];
        k++;
        if(k==4){
          j++;
          k=0;
        }
      }
       _pub_nrt_damp.write(tmp);     
    }

}

