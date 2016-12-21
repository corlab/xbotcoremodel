/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __X_BOT_CORE_MODEL_HPP__
#define __X_BOT_CORE_MODEL_HPP__

#include <IXBotModel.h>

#include <srdfdom_advr/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>

#include <yaml-cpp/yaml.h>

#define CHAIN_PER_GROUP 1

namespace XBot 
{
    class XBotCoreModel;
}

typedef std::map<int, std::string>  Rid2JointMap;
typedef std::map<std::string, int>  Joint2RidMap;

class XBot::XBotCoreModel : public srdf_advr::Model,
                            public XBot::IXBotModel
{
private:
    
    std::string urdf_string, srdf_string;
    std::string srdf_path;
    std::string joint_map_config_path;
    boost::shared_ptr<urdf::ModelInterface> urdf_model;
    KDL::Tree robot_tree;
    
        
    /**
     * @brief map between the chain name and the ids of the enabled joints in the chain 
     * 
     */
    std::map<std::string, std::vector<int>> robot;
    
    /**
     * @brief map between the chain name and the names of the enabled joints in the chain 
     * 
     */
    std::map<std::string, std::vector<std::string>> robot_string;
    
     /**
     * @brief map between the chain name and the id of the ft_sensors
     * 
     */
    std::map<std::string, int> ft_sensors;
    
    /**
     * @brief map between the chain name and the id of the imu_sensors
     * 
     */
    std::map<std::string, int> imu_sensors;
    
    /**
     * @brief map between joint robot id and joint name
     * 
     */
    Rid2JointMap rid2joint;
    
    /**
     * @brief map between joint name and joint robot id
     * 
     */
    Joint2RidMap joint2rid;
    
    // enabled joint number of the robot
    int joint_num = 0;
    
    // vector for the chain names
    std::vector<std::string> chain_names;
    
    // vector for the chain names
    std::vector<std::string> ordered_chain_names;
    
    // vector for the chain names of the legs ORDERED as the SRDF
    std::vector<std::string> legs_names;
    
    // vector for the chain names of the arms ORDERED as the SRDF
    std::vector<std::string> arms_names;
    
    // vector for the controlled joint name, i.e. joints that are labelled
    // as fixed in the URDF and, nonetheless, are controlled (e.g. hand motors)
    std::vector<std::string> controlled_joints;
    
    // vector for the disabled joints
    std::vector<std::string> disabled_joint_names;
    
    // map for the enabled joints in chains
    std::map<std::string, std::vector<std::string>> enabled_joints_in_chains;
    
    // map for the disabled joints in chains
    std::map<std::string, std::vector<std::string>> disabled_joints_in_chains;

    
    // TBD IMU

    
    
    /**
     * @brief load the URDF filename
     * 
     * @param filename the URDF filename
     * @return boost::shared_ptr< urdf::ModelInterface > the URDF model interface
     */
    boost::shared_ptr<urdf::ModelInterface> loadURDF(const std::string& filename);
    
    
    /**
     * @brief parse the SRDF in order to have the information of the robot chains and enabled/disabled joints.
     * 
     * @return bool true on success, false otherwise: one possible cause can be the incorrect SRDF format.
     */
    bool parseSRDF();
    
    /**
     * @brief get the enabled/disabled actuated (RotAxis or TransAxis) in the chain from base_link to tip_link
     * 
     * @param base_link base link of the chain
     * @param tip_link tip link of the chain
     * @param enabled_joints_in_chain vector that will be filled with the enabled joint names in the chain
     * @param disabled_joints_in_chain vector that will be filled with the disabled joint names in the chain
     * @return bool true on success, false if the chain does not exists in the robto tree
     */
    bool get_joints_in_chain( std::string base_link, 
                              std::string tip_link,
                              std::vector<std::string>& enabled_joints_in_chain,
                              std::vector<std::string>& disabled_joints_in_chain);

    
    
    void parseJointMap(void);

public:
    
    XBotCoreModel(void){}
    
    /**
     * @brief getter for the URDF ModelInterface
     * 
     * @return std::shared_ptr< urdf::ModelInterface > the URDF ModelInterface
     */
    boost::shared_ptr<urdf::ModelInterface const> get_urdf_model(void) const
    {
        return urdf_model; 
    }
    
    /**
     * @brief getter for the KDL robot tree
     * 
     * @return KDL::Tree the KDL robot tree
     */
    KDL::Tree get_robot_tree(void)
    { 
        return robot_tree;
    }
    
    /**
     * @brief getter for the chain names vector
     * 
     * @return std::vector< std::::string> the chain names vector
     */
    const std::vector<std::string>& get_chain_names(void) const 
    {
        return chain_names;
    }
    
    /**
     * @brief getter for the alphabetically ordered chain names vector
     * 
     * @return std::vector< std::::string> the chain names vector
     */
    const std::vector<std::string>& get_ordered_chain_names(void) const 
    {
        return ordered_chain_names;
    }
    
    /**
     * @brief initialization function for the model: it loads the URDF and parses the SRDF
     * 
     * @param urdf_filename URDF path
     * @param srdf_filename SRDF path
     * @param joint_map_config joint_map_config path
     * @return bool true on success, false otherwise
     */
    bool init(const std::string& urdf_filename, const std::string& srdf_filename,
              const std::string& joint_map_config);

    /**
     * @brief initialization function for the model: it loads the URDF and parses the SRDF
     *
     * @param urdf_filename URDF path
     * @param srdf_filename SRDF path
     * @return bool true on success, false otherwise
     */
    bool init(const std::string& urdf_filename, const std::string& srdf_filename);
    
    /**
     * @brief dynamically generate the robot as a map {chain_name} -> {vector of enabled joints}
     * 
     * @param  ...
     * @return void
     */
    void generate_robot(void);
    
    virtual std::map<std::string, std::vector<int>> get_robot(void) final
    {
        return robot;
    }
    
    virtual std::map<std::string,int> get_ft_sensors(void) final
    {
        return ft_sensors;
    }
    
    virtual std::map<std::string, int> get_imu_sensors() final
    {
        return imu_sensors;
    }
          
    virtual std::string rid2Joint(int rId) final
    {
        return rid2joint.find(rId) != rid2joint.end() ? rid2joint[rId] : ""; 
    }
    
    virtual int joint2Rid(std::string joint_name) final
    {
        return joint2rid.find(joint_name) != joint2rid.end() ? joint2rid[joint_name] : 0;
    }
    

    /**
     * @brief getter for the enabled joints vector in the chain
     *
     * @param chain_name the requested chain name
     * @param enabled_joints vector that will be filled with the enabled joint names
     * @return bool true if the chain exists, false otherwise
     */
    bool get_enabled_joints_in_chain( std::string chain_name, std::vector<std::string>& enabled_joints) const;


    /**
     * @brief getter for the disabled joints vector in the chain
     *
     * @param chain_name the requested chain name
     * @param disabled_joints vector that will be filled with the disabled joint names
     * @return bool true if the chain exists, false otherwise
     */
    bool get_disabled_joints_in_chain( std::string chain_name, std::vector<std::string>& disabled_joints) const;
    
    
    /**
     * @brief get the vector of the enabled joint names of the whole robot
     * 
     * @param joint_names array of the joint names filled by this function
     * @return void
     */
    void get_enabled_joint_names(std::vector<std::string>& joint_names) const;
    
    /**
     * @brief get the vector of the enabled joint ids of the whole robot
     * 
     * @param joint_names array of the joint ids filled by this function
     * @return void
     */
    void get_enabled_joint_ids(std::vector<int>& joint_ids) const;
    
    /**
     * @brief get the vector of the joint ids of the requested chain
     * 
     * @param chain_name the requested chain
     * @param joint_names array of the joint ids of the requested chain filled by this function
     * @return void
     */
    bool get_enabled_joint_ids_in_chain(std::string chain_name, std::vector<int>& joint_ids) const;
    
    /**
     * @brief get the number of joint in the robot
     * 
     * @return the number of joint of the robot
     */
    int get_joint_num() const;
    
    /**
     * @brief get the number of joint of a requested chain
     * 
     * @param chain_name the requested chain
     * @return the number of joint of the requested chain
     */
    int get_joint_num(std::string chain_name) const;
    
    /**
     * @brief Get the robot URDF as a string
     * 
     * @return URDF file as a const std::string&
     */
    const std::string& get_urdf_string() const;
    
    /**
     * @brief Get the robot SRDF as a string
     * 
     * @return SRDF file as a const std::string&
     */
    const std::string& get_srdf_string() const;
    
    /**
     * @brief Get the legs chain names ordered as in the SRDF
     * 
     * @return the legs chain names ordered as in the SRDF
     */
    const std::vector<std::string>& get_legs_chain() const;
    
    /**
     * @brief Get the arms chain names ordered as in the SRDF
     * 
     * @return the arms chain names ordered as in the SRDF
     */
    const std::vector<std::string>& get_arms_chain() const;
    
    bool check_joint_limits() const;
    
    ~XBotCoreModel() 
    {
    }
    
};

#endif //__X_BOT_CORE_MODEL_HPP__
