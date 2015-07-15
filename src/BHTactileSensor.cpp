/** \file BHTactileSensor.cpp
 * \brief OpenRAVE sensor plugin for the BarrettHand's tactile sensors.
 * \author Michael Koval
 * \date 2013
 */
#include <boost/assign/std/vector.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include "BHTactileSensor.h"

using namespace boost::assign;


BHTactileSensor::BHTactileSensor(OpenRAVE::EnvironmentBasePtr env, OpenRAVE::RobotBasePtr robot,
                                 std::string const &ns, std::string const &link_prefix)
    : OpenRAVE::SensorBase(env)
    , owd_ns_(ns)
    , robot_(robot)
{
    // Load the BarrettHand's tactile pad configuration file.
    std::string const config_path = ros::package::getPath("or_owd_controller") + "/config/tactile.yaml";
    loadGeometry(config_path, link_prefix, cell_offsets_);

    ros::NodeHandle nh_owd(owd_ns_);
    sub_tactile_ = nh_owd.subscribe("tactile", 1, &BHTactileSensor::tactileCallback, this);

    // FIXME: This is a hack to work around the lack of a SetName function in Python.
    SetName("BHTactileSensor");
}

int BHTactileSensor::Configure(OpenRAVE::SensorBase::ConfigureCommand, bool blocking)
{
    return 0;
}

OpenRAVE::SensorBase::SensorGeometryPtr BHTactileSensor::GetSensorGeometry(SensorType type)
{
    boost::shared_ptr<TactileGeomData> geom = boost::make_shared<TactileGeomData>();
    geom->positions.resize(cell_offsets_.size());
    geom->thickness = 0;

    for (size_t i = 0; i < geom->positions.size(); ++i) {
        geom->positions[i] = transformGeometry(i);
    }
    return geom;
}

OpenRAVE::SensorBase::SensorDataPtr BHTactileSensor::CreateSensorData(OpenRAVE::SensorBase::SensorType type)
{
    return boost::make_shared<OpenRAVE::SensorBase::TactileSensorData>();
}

bool BHTactileSensor::GetSensorData(OpenRAVE::SensorBase::SensorDataPtr sensor_data)
{
    if (sensor_data->GetType() != OpenRAVE::SensorBase::ST_Tactile) {
        throw OpenRAVE::openrave_exception("SensorData must be for type ST_Tactile.",
                                           OpenRAVE::ORE_InvalidArguments);
    } else if (!current_tactile_) {
        RAVELOG_ERROR("There is no tactile data available.\n");
        return false;
    } 
    // Verify that the data includes the correct number of cells.
    size_t const total_size = current_tactile_->finger1.size() + current_tactile_->finger2.size()
                            + current_tactile_->finger3.size() + current_tactile_->palm.size();
    if (total_size != cell_offsets_.size()) {
        throw OpenRAVE::openrave_exception(boost::str(
            boost::format("Incorrect number of cells: expected %d, got %d.")
                % cell_offsets_.size() % total_size),
            OpenRAVE::ORE_Failed);
    }

    boost::shared_ptr<TactileSensorData> tactile = boost::dynamic_pointer_cast<TactileSensorData>(sensor_data);
    tactile->forces.clear();
    tactile->forces.reserve(total_size);
    std::fill(tactile->force_covariance.begin(), tactile->force_covariance.end(), 0.0);

    // TODO: Return proper normals.
    size_t cell_index = 0;
    BOOST_FOREACH (float const pressure, current_tactile_->palm) {
        tactile->forces += transformData(cell_index, pressure);
        cell_index++;
    }
    BOOST_FOREACH (float const pressure, current_tactile_->finger1) {
        tactile->forces += transformData(cell_index, pressure);
        cell_index++;
    }
    BOOST_FOREACH (float const pressure, current_tactile_->finger2) {
        tactile->forces += transformData(cell_index, pressure);
        cell_index++;
    }
    BOOST_FOREACH (float const pressure, current_tactile_->finger3) {
        tactile->forces += transformData(cell_index, pressure);
        cell_index++;
    }
    return true;
}

bool BHTactileSensor::Supports(OpenRAVE::SensorBase::SensorType type)
{
    return type == OpenRAVE::SensorBase::ST_Tactile;
}

void BHTactileSensor::SetTransform(OpenRAVE::Transform const &trans)
{
    transform_ = trans;
}

OpenRAVE::Transform BHTactileSensor::GetTransform()
{
    // TODO: Populate this from TF.
    return transform_;
}

bool BHTactileSensor::SimulationStep(OpenRAVE::dReal fTimeElapsed)
{
    queue_.callAvailable();
    return true;
}

void BHTactileSensor::loadGeometry(std::string const &path, std::string const &link_prefix,
                                   std::vector<TactileCell> &cells) const
{
    std::ifstream stream(path.c_str());
    YAML::Node yaml;
    try {
        yaml = YAML::Load(stream);
    } catch(YAML::ParserException& e) {
        throw OPENRAVE_EXCEPTION_FORMAT("Unable to load tactile sensor geometry from '%s'.",
                                        path.c_str(), OpenRAVE::ORE_Failed);
    }

    for (size_t j = 0; j < yaml.size(); ++j) {
        std::string parent_name;
        parent_name = yaml[j]["parent"].as<std::string>();
        std::string const full_parent_name = link_prefix + parent_name;

        OpenRAVE::RobotBase::LinkPtr parent = robot_->GetLink(full_parent_name);
        if (!parent) {
            throw OPENRAVE_EXCEPTION_FORMAT("There is no link named '%s'.",
                                            full_parent_name.c_str(), OpenRAVE::ORE_Failed);
        }

        // Load the position and normal vector.
        YAML::Node const &cells_yaml = yaml[j]["cells"];
        for (size_t i = 0; i < cells_yaml.size(); ++i) {
            YAML::Node const &cell_yaml = cells_yaml[i];
            YAML::Node const &position_yaml = cell_yaml[0];
            YAML::Node const &normal_yaml = cell_yaml[1];

            TactileCell cell;
            cell.parent = parent;
            cell.position[0] = position_yaml[0].as<double>();
            cell.position[1] = position_yaml[1].as<double>();
            cell.position[2] = position_yaml[2].as<double>();
            cell.normal[0] = normal_yaml[0].as<double>();
            cell.normal[1] = normal_yaml[1].as<double>();
            cell.normal[2] = normal_yaml[2].as<double>();

            cell.normal.normalize();
            cells += cell;
        }
    }
}

OpenRAVE::Vector BHTactileSensor::transformData(size_t index, double pressure) const
{
    TactileCell const &cell = cell_offsets_[index];
    OpenRAVE::Transform const parent_transform = cell.parent->GetTransform();
    return pressure * parent_transform.rotate(cell.normal);
}

OpenRAVE::Vector BHTactileSensor::transformGeometry(size_t index) const
{
    TactileCell const &cell = cell_offsets_[index];
    OpenRAVE::Transform const parent_transform = cell.parent->GetTransform();
    return parent_transform * cell.position;
}

void BHTactileSensor::tactileCallback(owd_msgs::BHTactileConstPtr tactile_msg)
{
    current_tactile_ = tactile_msg;
}
