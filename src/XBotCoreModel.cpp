#include <XBotCoreModel.h>
#include <stdio.h>

#ifdef __XENO__
     #include <rtdk.h>
     #define DPRINTF rt_printf
#else
     #include <stdio.h>
     #define DPRINTF printf
#endif

boost::shared_ptr<urdf::ModelInterface> XBot::XBotCoreModel::loadURDF(const std::string& filename)
{

    // get the entire file
    std::string xml_string;
    std::fstream xml_file(filename.c_str(), std::fstream::in);
    if (xml_file.is_open())
    {
        while (xml_file.good())
        {
        std::string line;
        std::getline( xml_file, line);
        xml_string += (line + "\n");
        }
        xml_file.close();

        return urdf::parseURDF(xml_string);
    }
    else
    {
        throw std::runtime_error("Could not open file " + filename + " for parsing.");
        return boost::shared_ptr<urdf::ModelInterface>();
    }
}

bool XBot::XBotCoreModel::parseSRDF() {

    std::vector<Group> actual_groups = getGroups();
    std::vector<DisabledJoint> actual_disabled_joints = getDisabledJoints();

    int group_num = actual_groups.size();

    // NOTE the last group is the "chains" group
    int chain_num = actual_groups[group_num - 1].subgroups_.size();
    chain_names.resize(chain_num);
    // fill the chain names vector
    for(int i = 0; i < chain_num; i++) {
        chain_names[i] = (actual_groups[group_num - 1].subgroups_[i]);
    }

    // put the disabled joint in the disabled_joint_names data structure
    int disabled_joint_num = actual_disabled_joints.size();
    disabled_joint_names.resize(disabled_joint_num);
    for( int i = 0; i < disabled_joint_num; i++) {
       disabled_joint_names.push_back(actual_disabled_joints[i].name_);
    }

    // iterate over the groups, without the last one -> it is the "chains" group
    for(int i = 0; i < group_num - 1; i++) {
        // if the group represents a kinematic chain
        if(std::find(chain_names.begin(), chain_names.end(), actual_groups[i].name_) != chain_names.end() ) {
            // check the number of chain per group: it has to be 1
            if( actual_groups[i].chains_.size() != CHAIN_PER_GROUP)  {
                // Error: only one chain per group
                DPRINTF("ERROR: for the kinematic chain groups you can specify only one chain per group in your SRDF ( %s ): check %s group\n", srdf_path.c_str(), actual_groups[i].name_.c_str() );
                return false;
            }
            // fill the enabled/disabled joints in chain map
            if( !get_joints_in_chain( actual_groups[i].chains_[0].first,
                                      actual_groups[i].chains_[0].second,
                                      enabled_joints_in_chains[actual_groups[i].name_],
                                      disabled_joints_in_chains[actual_groups[i].name_]) ) {

                DPRINTF("ERROR: get_joints_in_chain() failed\n");
                return false;
            }

        }
        // NOTE not a kinematic chain : check for FT 
        else {
            if(actual_groups[i].name_ == "force_torque_sensors") {
                for(int j = 0; j < actual_groups[i].joints_.size(); j++) {
                    ft_sensors[actual_groups[i].joints_[j]] = joint2Rid(actual_groups[i].joints_[j]);
                }
            }
            
        }
        // TBD IMU

    }
    return true;
}

bool XBot::XBotCoreModel::get_joints_in_chain(  std::string base_link,
                                                std::string tip_link,
                                                std::vector<std::string>& enabled_joints_in_chain,
                                                std::vector<std::string>& disabled_joints_in_chain)
{

    KDL::Chain actual_chain;
    if( robot_tree.getChain(base_link, tip_link, actual_chain) ) {
        int segments_num = actual_chain.getNrOfSegments();
        for( int i = 0; i < segments_num; i++) {
            KDL::Segment actual_segment = actual_chain.getSegment(i);
            KDL::Joint actual_joint = actual_segment.getJoint();

            // if the joint is revolute or prismatic
            if ( actual_joint.getTypeName() == "RotAxis"   ||
                 actual_joint.getTypeName() == "TransAxis" ||  // TBD check this if needed
                 actual_joint.getName() == "l_handj" || actual_joint.getName() == "r_handj") {   // TBD check the model for the hands

                // if the joint is enabled
                if( !(std::find(disabled_joint_names.begin(), disabled_joint_names.end(), actual_joint.getName()) != disabled_joint_names.end() ) ) {
                    enabled_joints_in_chain.push_back(actual_joint.getName());
                }
                // disabled joint
                else {
                    disabled_joints_in_chain.push_back(actual_joint.getName());
                }
            }
        }

        return true;
    }

    // chain not found
    return false;
}

bool XBot::XBotCoreModel::get_enabled_joints_in_chain( std::string chain_name, std::vector<std::string>& enabled_joints) const
{
    // check if the chain exists
    if( enabled_joints_in_chains.count(chain_name) ) {
        enabled_joints = enabled_joints_in_chains.at(chain_name);
        return true;
    }

    // chain does not exists
    DPRINTF("ERROR: requested chain in get_enabled_joints_in_chain() does not exist.\n");
    return false;
}

bool XBot::XBotCoreModel::get_disabled_joints_in_chain( std::string chain_name, std::vector<std::string>& disabled_joints) const
{
    // check if the chain exists
    if( enabled_joints_in_chains.count(chain_name) ) {
        disabled_joints = enabled_joints_in_chains.at(chain_name);
        return true;
    }

    // chain does not exists
    DPRINTF("ERROR: requested chain in get_disabled_joints_in_chain() does not exist.\n");
    return false;
}

void XBot::XBotCoreModel::parseJointMap(void)
{
    // read the joint map config file -> we choose to separate it from the one used by XBotCore and ec_boards_iface
    YAML::Node joint_map_cfg = YAML::LoadFile(joint_map_config_path);
    const YAML::Node& joint_map = joint_map_cfg["joint_map"];

    // iterate over the node
    for(YAML::const_iterator it=joint_map.begin();it != joint_map.end();++it) {
        int tmp_rid = it->first.as<int>();
        std::string tmp_joint = it->second.as<std::string>();
        // fill the maps
        rid2joint[tmp_rid] = tmp_joint;
        joint2rid[tmp_joint] = tmp_rid;

//         DPRINTF("rid2joint : rid -> %d ==> joint -> %s\n", tmp_rid, rid2joint[tmp_rid].c_str());
//         DPRINTF("joint2rid : joint -> %s ==> rid -> %d\n", tmp_joint.c_str(), joint2rid[tmp_joint]);
    }
}

bool XBot::XBotCoreModel::init(const std::string& urdf_filename, 
                               const std::string& srdf_filename, 
                               const std::string& joint_map_config)
{
    // SRDF path
    srdf_path = srdf_filename;

    // joint_map_config path
    joint_map_config_path = joint_map_config;

    // load URDF model from file
    urdf_model = loadURDF(urdf_filename);

    // parse the Joint map
    parseJointMap();

    // create the robot KDL tree from the URDF model
    if( !kdl_parser::treeFromUrdfModel(*urdf_model, robot_tree) ) {
        DPRINTF("Failed to construct kdl tree");
        return false;
    }

    // load SRDF model from file
    bool ret = this->initFile(*urdf_model, srdf_filename);

    // parse the SRDF file and fill the data structure
    return (ret && parseSRDF());
}

bool XBot::XBotCoreModel::init(const std::string& urdf_filename, const std::string& srdf_filename)
{
    // SRDF path
    srdf_path = srdf_filename;

    // joint_map_config path
    joint_map_config_path = "";

    // load URDF model from file
    urdf_model = loadURDF(urdf_filename);

    // parse the Joint map
    //parseJointMap();

    // create the robot KDL tree from the URDF model
    if( !kdl_parser::treeFromUrdfModel(*urdf_model, robot_tree) ) {
        DPRINTF("Failed to construct kdl tree");
        return false;
    }

    // load SRDF model from file
    bool ret = this->initFile(*urdf_model, srdf_filename);

    // parse the SRDF file and fill the data structure
    return (ret && parseSRDF());
}

void XBot::XBotCoreModel::generate_robot(void)
{
    // generate the robot : map between tha chain name and the joint ids of the chain
    std::vector<std::string> actual_chain_names = get_chain_names();
    for( int i = 0; i < actual_chain_names.size(); i++) {
        std::vector<std::string> enabled_joints_name_aux;
        std::vector<int> enabled_joints_id_aux;
        if( get_enabled_joints_in_chain(actual_chain_names[i], enabled_joints_name_aux) ) {
            for( int j = 0; j < enabled_joints_name_aux.size(); j++ ) {
                if( joint2rid.count(enabled_joints_name_aux[j]) ) {
                    enabled_joints_id_aux.push_back(joint2rid.at(enabled_joints_name_aux[j]));
                }
            }
            robot[actual_chain_names[i]] = enabled_joints_id_aux;
            joint_num += enabled_joints_id_aux.size();
        }
    }
}

bool XBot::XBotCoreModel::get_enabled_joint_ids_in_chain(std::string chain_name, std::vector< int >& joint_ids) const
{
    // check if the chain exists
    if( robot.count(chain_name) ) {
        joint_ids = robot.at(chain_name);
        return true;
    }

    // chain does not exists
    DPRINTF("ERROR: requested chain in get_enabled_joints_in_chain() does not exist.\n");
    return false;
}


void XBot::XBotCoreModel::get_enabled_joint_ids(std::vector< int >& joint_ids) const
{
    joint_ids.clear();
    for(const auto& c : robot) {
        joint_ids.insert( joint_ids.end(), c.second.begin(), c.second.end() );
    }
}


void XBot::XBotCoreModel::get_enabled_joint_names(std::vector< std::string >& joint_names) const
{
    joint_names.clear();
    for(const auto& c : enabled_joints_in_chains) {
        joint_names.insert( joint_names.end(), c.second.begin(), c.second.end() );
    }
}


int XBot::XBotCoreModel::get_joint_num(std::string chain_name) const
{
    // check if the chain exists
    if( enabled_joints_in_chains.count(chain_name) ) {
        return enabled_joints_in_chains.at(chain_name).size();
    }
    else {
        DPRINTF("ERROR: requested chain in get_joint_num() does not exist.\n");
        return -1;
    }
}


int XBot::XBotCoreModel::get_joint_num() const
{
    return joint_num;
}



