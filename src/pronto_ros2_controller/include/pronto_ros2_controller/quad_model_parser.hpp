#include "urdf/model.h"


#include <string>
#include <vector>
#include <array>
#include <map>




namespace pronto_controller
{

    const std::vector<std::string> quadruped_leg_id =
    {
        "LF",
        "LH",
        "RF",
        "RH"
    };

    const std::vector<std::string> quadruped_jnt_id = 
    {
        "HAA",
        "HFE",
        "KFE"
    };

    enum Axis_Direction
    {
        x,y,z,all,error,not_init
    };

    // this class will be used by the legodom manager to adapt the feet_contact_force and the velocity correction function to the 
    // desired model
    class Model_Parser
    {
        public:
            // build the model from urdf file 
            Model_Parser(std::string urdf_path);
            
            ~Model_Parser(){};

            Axis_Direction get_ker_dir();

            int get_robot_DOF()
            {
                if(urdf_usable_)
                    return jnts_find_*quadruped_leg_id.size();
                else
                {
                    throw std::logic_error("NOT VALID MODEL");
                    return 0;
                }
            }


        protected:

            void get_jnt_list()
            {
                auto all_jnt = model_.joints_;
                urdf::Vector3 axis;
                std::array<double,3> ax;
                for(auto &jnt:all_jnt)
                {
                    axis = jnt.second->axis;
                    if(axis.x != 0.0 || axis.y != 0.0 || axis.z != 0.0)
                    {
                        jnt_names_.push_back(jnt.first);
                        ax = {axis.x,axis.y,axis.z};
                        jnt_ptr_.push_back(ax);
                    }
                }
            }


        private:

            urdf::Model model_;
            
            bool urdf_usable_ = false;

            std::vector<std::string> jnt_names_;
            std::vector<std::array<double,3>> jnt_ptr_;

            int jnts_find_ = 0;
            std::map<std::string,std::map<std::string,std::array<double,3>>> kin_tree_;


    };
};