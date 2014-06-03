/*
* Copyright © 2014 Simple Entertainment Limited
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
#ifndef PHYSXBODY_H_
#define PHYSXBODY_H_

#include <memory>

#include <PxPhysicsAPI.h>

#include <simplicity/physics/Body.h>

namespace simplicity
{
	namespace physx
	{
		class PhysXBody : public Body
		{
			public:
				PhysXBody(::physx::PxPhysics& physics, ::physx::PxCooking& cooking, const Material& material, Model* model,
					const Matrix44& transform, bool dynamic);

				void applyForce(const Vector3& force, const Vector3& position) override;

				void applyTorque(const Vector3& torque) override;

				void clearForces() override;

				::physx::PxActor* getActor();

				Vector3 getLinearVelocity() const override;

				const Material& getMaterial() const override;

				const Model* getModel() const override;

				::physx::PxGeometry* getPhysXModel();

				bool isDynamic() override;

				void setDynamic(bool dynamic) override;

				void setLinearVelocity(const Vector3& linearVelocity) override;

				void setMaterial(const Material& material) override;

			private:
				::physx::PxActor* actor;

				bool dynamic;

				Material material;

				Model* model;

				::physx::PxMaterial* physxMaterial;

				std::unique_ptr<::physx::PxGeometry> physxModel;

				void createPhysXModel(::physx::PxPhysics& physics, ::physx::PxCooking& cooking);
		};
	}
}

#endif /* PHYSXBODY_H_ */
