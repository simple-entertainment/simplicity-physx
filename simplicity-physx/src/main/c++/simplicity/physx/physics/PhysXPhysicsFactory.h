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
#ifndef PHYSXPHYSICSFACTORY_H_
#define PHYSXPHYSICSFACTORY_H_

#include <PxPhysicsAPI.h>

#include <simplicity/physics/PhysicsFactory.h>

namespace simplicity
{
	namespace simphysx
	{
		class PhysXPhysicsFactory : public PhysicsFactory
		{
		public:
			PhysXPhysicsFactory(physx::PxPhysics* physics, physx::PxCooking* cooking);

			std::unique_ptr<Body> createBody(const Body::Material& material, Model* model, const Matrix44& transform,
				bool dynamic) override;

		private:
			physx::PxCooking* cooking;

			physx::PxPhysics* physics;
		};
	}
}

#endif /* PHYSXPHYSICSFACTORY_H_ */
