/*
 * groundManager.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: chpark
 */
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/point_to_triangle_projection.h>
#include <limits>

namespace itomp_cio_planner
{

GroundManager GroundManager::instance_;

GroundManager::GroundManager()
{
}

GroundManager::~GroundManager()
{
}

void GroundManager::init()
{
}

double interpolateSqrt(double x, double x1, double x2, double y1, double y2)
{
	//double y = (y2 - y1) * sqrt((x - x1) / (x2 - x1)) + y1;
	double y = (y2 - y1) * (x - x1) / (x2 - x1) + y1;
	return y;
}

void GroundManager::getNearestGroundPosition(const KDL::Vector& in, KDL::Vector& out, KDL::Vector& normal, const planning_scene::PlanningScenePtr& planning_scene) const
{
    //TEMP HARDCODING FOR SWIMMING POOL SCENARIO
   /* shapes::Plane planes[3];

    planes[0] = shapes::Plane(0, 0, 1, 0);
    planes[1] = shapes::Plane(0, 1, 0, -4);
    planes[2] = shapes::Plane(0, -0.074, -1, -2.8);

    KDL::Vector normals[3];

    KDL::Vector proj_points[3];
    for (int i = 0; i < 3; ++i)
    {
        normals[i] = KDL::Vector(planes[i].a, planes[i].b, planes[i].c);
        double norm = normals[i].Norm();
        normals[i].Normalize();
        double lambda = -dot(normals[i], in) + (planes[i].d/norm);
        proj_points[i] = lambda * normals[i] + in;
    }

    if (proj_points[0].y() < 4)
        proj_points[0].y(4);
    if (proj_points[1].z() > 0)
        proj_points[1].z(0);
    if (proj_points[2].y() > 4)
    {
        proj_points[2].y(4);
        proj_points[2].z(-0.074 * 4 - 2.8);
    }

    double max_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < 3; ++i)
    {
        double dist = (in - proj_points[i]).Norm();
        if (dist < max_dist)
        {
            out = proj_points[i];
            normal = normals[i];
            max_dist = dist;
        }
    }*/



    normal = KDL::Vector(0, 0, 1);
    double current_min_distance = std::numeric_limits<double>::max();
    Eigen::Vector3d in_eigen(in.x(), in.y(), in.z());
    const collision_detection::WorldConstPtr& world = planning_scene->getWorld();
    std::vector<std::string> object_ids = world->getObjectIds();
    for (int i = 0; i < object_ids.size(); ++i)
    {
        collision_detection::World::ObjectConstPtr obj = world->getObject(object_ids[i]);
        for (int j = 0; j < obj->shapes_.size(); ++j)
        {
            shapes::ShapeConstPtr shape = obj->shapes_[j];
            const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(shape.get());
            if (mesh == NULL)
                continue;
            Eigen::Affine3d& transform = obj->shape_poses_[j];
           for(int k=0; k<mesh->triangle_count; ++k)
            {
                int triangle_vertex1 = mesh->triangles[3*k];
                int triangle_vertex2 = mesh->triangles[3*k+1];
                int triangle_vertex3 = mesh->triangles[3*k+2];
                Eigen::Vector3d position1(mesh->vertices[3*triangle_vertex1],mesh->vertices[3*triangle_vertex1+1],mesh->vertices[3*triangle_vertex1+2]);
                Eigen::Vector3d position2(mesh->vertices[3*triangle_vertex2],mesh->vertices[3*triangle_vertex2+1],mesh->vertices[3*triangle_vertex2+2]);
                Eigen::Vector3d position3(mesh->vertices[3*triangle_vertex3],mesh->vertices[3*triangle_vertex3+1],mesh->vertices[3*triangle_vertex3+2]);
                position1 = transform * position1;
                position2 = transform * position2;
                position3 = transform * position3;

                Eigen::Vector3d projection = ProjPoint2Triangle(position1, position2, position3, in_eigen);
                double distance = (in_eigen - projection).norm();
                if(distance < current_min_distance)
                {
                    current_min_distance = distance;
                    Eigen::Vector3d normal_eigen(mesh->triangle_normals[3*k], mesh->triangle_normals[3*k+1], mesh->triangle_normals[3*k+2]);
                    normal_eigen = transform.rotation() * normal_eigen;
                    normal.x(normal_eigen.x());
                    normal.y(normal_eigen.y());
                    normal.z(normal_eigen.z());
                    out.x(projection.x());
                    out.y(projection.y());
                    out.z(projection.z());
                }
            }
        }

    }
}

void GroundManager::getSafeGroundPosition(const KDL::Vector& in, KDL::Vector& out) const
{
	const double FOOT_FRONT = 0.2;
	const double FOOT_REAR = 0.05;
	const double MARGIN = 0.1;

	double safeX = in.x();
	double height = 0.0;
	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 1.0)
	{
		const int NUM_TERRAINS = 7;

		double x[] =
		{ -50.0, -1.0, -0.6, -0.2, 0.2, 0.6, 1.0, 50.0 };
		double z[] =
		{ 0.0, 0.15, 0.3, 0.45, 0.3, 0.15, 0.0, 0.0 };

		for (int i = 0; i < NUM_TERRAINS; ++i)
		{
			if (in.x() >= x[i] && in.x() <= x[i + 1])
			{
				height = z[i];

				if (z[i] < z[i + 1] && x[i + 1] - FOOT_FRONT - MARGIN < in.x())
					safeX = x[i + 1] - FOOT_FRONT - MARGIN;
				if (i != 0 && z[i - 1] < z[i] && in.x() < x[i] + FOOT_REAR)
				{
					safeX = x[i] - FOOT_FRONT - MARGIN;
					height = z[i - 1];
				}

				if (z[i] > z[i + 1] && x[i + 1] - FOOT_FRONT < in.x())
					safeX = x[i + 1] - FOOT_FRONT;
				if (i != 0 && z[i - 1] > z[i] && in.x() < x[i] + FOOT_REAR + MARGIN)
				{
					safeX = x[i] - FOOT_FRONT;
					height = z[i - 1];
				}
			}
		}
	}
	else if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 2.0)
	{
		if (-1.60 - FOOT_FRONT - MARGIN < in.x() && in.x() < -1.50 + FOOT_REAR + MARGIN)
		{
			safeX = -1.60 - FOOT_FRONT - MARGIN;
		}
	}
	else if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 3.0)
	{
		height = 0.0;
	}

	out = KDL::Vector(safeX, in.y(), height);
}

}

