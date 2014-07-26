/*
* Copyright � 2014 Simple Entertainment Limited
*
* This file is part of The Simplicity Engine.
*
* The Simplicity Engine is free software: you can redistribute it and/or modify it under the terms of the GNU General
* Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option)
* any later version.
*
* The Simplicity Engine is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
* details.
*
* You should have received a copy of the GNU General Public License along with The Simplicity Engine. If not, see
* <http://www.gnu.org/licenses/>.
*/
#include <simplicity/model/Mesh.h>
#include <simplicity/model/Plane.h>
#include <simplicity/model/shape/Cube.h>
#include <simplicity/model/shape/Sphere.h>

#include "../math/PhysXMatrix.h"
#include "../math/PhysXVector.h"
#include "PhysXBody.h"

using namespace physx;
using namespace std;

namespace simplicity
{
	namespace simphysx
	{
		PhysXBody::PhysXBody(PxPhysics& physics, PxCooking& cooking, const Material& material, Model* model,
			const Matrix44& transform, bool dynamic) :
			actor(nullptr),
			dynamic(dynamic),
			material(material),
			model(model),
			physxMaterial(nullptr),
			physxModel()
		{
			physxMaterial = physics.createMaterial(material.friction, material.friction, material.restitution);

			createPhysXModel(physics, cooking);

			if (dynamic)
			{
				actor = PxCreateDynamic(physics, PhysXMatrix::toPxTransform(transform), *physxModel, *physxMaterial,
					material.density);
				PxRigidBodyExt::updateMassAndInertia(*static_cast<PxRigidBody*>(actor), &material.density, 1);
			}
			else
			{
				actor = PxCreateStatic(physics, PhysXMatrix::toPxTransform(transform), *physxModel, *physxMaterial);
			}
		}

		void PhysXBody::applyForce(const Vector3& force, const Vector3&)
		{
			PxRigidBody* rigidBody = actor->isRigidBody();
			if (rigidBody != nullptr)
			{
				rigidBody->addForce(PhysXVector::toPxVec3(force));
			}
		}

		void PhysXBody::applyTorque(const Vector3& torque)
		{
			PxRigidBody* rigidBody = actor->isRigidBody();
			if (rigidBody != nullptr)
			{
				rigidBody->addTorque(PhysXVector::toPxVec3(torque));
			}
		}

		void PhysXBody::clearForces()
		{
			PxRigidBody* rigidBody = actor->isRigidBody();
			if (rigidBody != nullptr)
			{
				rigidBody->clearForce();
			}
		}

		void PhysXBody::createPhysXModel(PxPhysics& physics, PxCooking& cooking)
		{
			Plane* plane = dynamic_cast<Plane*>(model);
			if (plane != nullptr)
			{
				actor = PxCreatePlane(physics, PxPlane(PhysXVector::toPxVec3(plane->getPositionOnPlane()),
					PhysXVector::toPxVec3(plane->getNormal())), *physxMaterial);
				return;
			}

			Cube* cube = dynamic_cast<Cube*>(model);
			if (cube != nullptr)
			{
				physxModel.reset(new PxBoxGeometry(cube->getHalfEdgeLength(), cube->getHalfEdgeLength(),
					cube->getHalfEdgeLength()));
			}
			Mesh* mesh = dynamic_cast<Mesh*>(model);
			if (mesh != nullptr)
			{
				// Need a concurrent block of data...
				// TODO Try this without conversion...
				const MeshData& meshData = mesh->getData();

				vector<PxVec3> convexMeshData;
				convexMeshData.reserve(meshData.size());

				for (unsigned int index = 0; index < meshData.size(); index++)
				{
					PxVec3 vertex;
					vertex.x = meshData[index].position.X();
					vertex.y = meshData[index].position.Y();
					vertex.z = meshData[index].position.Z();
					convexMeshData.push_back(vertex);
				}

				PxConvexMeshDesc convexMeshDesc;
				convexMeshDesc.points.count = convexMeshData.size();
				convexMeshDesc.points.stride = sizeof(PxVec3);
				convexMeshDesc.points.data = &convexMeshData[0];
				convexMeshDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

				// Roasting...
				PxDefaultMemoryOutputStream buffer;
				cooking.cookConvexMesh(convexMeshDesc, buffer);
				PxDefaultMemoryInputData input(buffer.getData(), buffer.getSize());

				physxModel.reset(new PxConvexMeshGeometry(physics.createConvexMesh(input)));
			}
			const Sphere* sphere = dynamic_cast<const Sphere*>(model);
			if (sphere != nullptr)
			{
				physxModel.reset(new PxSphereGeometry(sphere->getRadius()));
			}
		}

		PxActor* PhysXBody::getActor()
		{
			return actor;
		}

		Vector3 PhysXBody::getLinearVelocity() const
		{
			PxRigidBody* rigidBody = actor->isRigidBody();
			if (rigidBody != nullptr)
			{
				return PhysXVector::toVector3(rigidBody->getLinearVelocity());
			}

			return Vector3(0.0f, 0.0f, 0.0f);
		}

		const Body::Material& PhysXBody::getMaterial() const
		{
			return material;
		}

		const Model* PhysXBody::getModel() const
		{
			return model;
		}

		PxGeometry* PhysXBody::getPhysXModel()
		{
			return physxModel.get();
		}

		bool PhysXBody::isDynamic()
		{
			return dynamic;
		}

		void PhysXBody::setDynamic(bool /* dynamic */)
		{
		}

		void PhysXBody::setLinearVelocity(const Vector3& linearVelocity)
		{
			PxRigidBody* rigidBody = actor->isRigidBody();
			if (rigidBody != nullptr)
			{
				rigidBody->setLinearVelocity(PhysXVector::toPxVec3(linearVelocity));
			}
		}

		void PhysXBody::setMaterial(const Material&)
		{
		}
	}
}
