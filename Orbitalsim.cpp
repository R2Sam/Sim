#include "Orbitalsim.h"

constexpr double Gkm = 6.67430e-20;

Orbitalbody::Orbitalbody(const std::string& name, const bool craft, const double mass, const double radius, const Vector3d position, const Vector3d velocity) :
name(name),
craft(craft),
mass(mass),
radius(radius),
position(position),
velocity(velocity)
{

}

void Orbitalbody::UpdatePosition(const double dt)
{
	position += velocity * dt + (acceleration + thrust) * 0.5 * dt * dt;
	oldAcceleration = acceleration;
	acceleration = Vector3dZero();
}

void Orbitalbody::UpdateVelocity(const double dt)
{
	velocity += 0.5 * (oldAcceleration + thrust + acceleration + thrust) * dt;
}

Orbitalsim::Orbitalsim(const std::vector<Orbitalbody>& bodies) :
_bodies(bodies)
{
	
}

Orbitalsim::~Orbitalsim()
{

}

void Orbitalsim::Update(const double timeS, const double dt)
{
	int loops = (int)(timeS / dt);
	double remainder = std::fmod(timeS, dt);
	if (std::abs(remainder) < 1e-9)
	{
		remainder = 0;
	}

	double timeStep = dt;

	if (loops < 0)
	{
		loops = -loops;
		timeStep = -timeStep;
		remainder = -remainder;
	}

	for (int i = 0; i < loops; i++)
	{
		Update(_bodies, timeStep);
	}

	if (remainder)
	{
		Update(_bodies, remainder);
	}

	GetParents(_bodies);
}

std::pair<std::vector<Orbitalbody>, std::unordered_map<std::string, Orbitalbody>> Orbitalsim::SoftUpdate(const double timeS, const double dt)
{
	std::vector<Orbitalbody> bodies = _bodies;

	int loops = (int)(timeS / dt);
	double remainder = std::fmod(timeS, dt);
	if (std::abs(remainder) < 1e-9)
	{
		remainder = 0;
	}

	double timeStep = dt;

	if (loops < 0)
	{
		loops = -loops;
		timeStep = -timeStep;
		remainder = -remainder;
	}

	for (int i = 0; i < loops; i++)
	{
		Update(bodies, timeStep);
	}

	if (remainder)
	{
		Update(bodies, remainder);
	}

	GetParents(bodies);

	std::unordered_map<std::string, Orbitalbody> map;

	for (const Orbitalbody& body: bodies)
	{
		map[body.name] = body;
	}

	return {bodies, map};
}

void Orbitalsim::SetBodies(const std::vector<Orbitalbody>& bodies)
{
	_bodies = bodies;
}

std::vector<Orbitalbody> Orbitalsim::GetBodies()
{
	return _bodies;
}

void Orbitalsim::SetBodiesMap(const std::unordered_map<std::string, Orbitalbody>& bodiesMap) 
{
	std::vector<Orbitalbody> bodies;
	bodies.reserve(bodiesMap.size());

	for (const auto& pair : bodiesMap)
	{
		bodies.emplace_back(pair.second);
	}

	_bodies = bodies;
}

std::unordered_map<std::string, Orbitalbody> Orbitalsim::GetBodiesMap()
{
	std::unordered_map<std::string, Orbitalbody> map;

	for (const Orbitalbody& body: _bodies)
	{
		map[body.name] = body;
	}

	return map;
}

bool Orbitalsim::EditBody(const Orbitalbody& body)
{
	std::unordered_map<std::string, Orbitalbody> map = GetBodiesMap();

	auto it = map.find(body.name);
	if (it == map.end())
	{
		return false;
	}

	it->second = body;

	SetBodiesMap(map);

	return true;
}

void Orbitalsim::Update(std::vector<Orbitalbody>& bodies, const double dt)
{
	#pragma omp parallel for
	for (Orbitalbody& body : bodies)
	{
		body.UpdatePosition(dt);
	}

	#pragma omp parallel for
	for (int i = 0; i < bodies.size(); i++)
	{
		for (int j = i + 1; j < bodies.size(); j++)
		{
			Vector3d r = bodies[j].position - bodies[i].position;
			double lengthSqr = r.lengthSqr();
			Vector3d tmp = r / (std::max(lengthSqr, 1.1920929E-7) * std::sqrt(lengthSqr));

			if (!bodies[j].craft)
			{
				#pragma omp critical
				bodies[i].acceleration += Gkm * bodies[j].mass * tmp;
			}
			#pragma omp critical
			bodies[j].acceleration -= Gkm * bodies[i].mass * tmp;
		}
	}

	#pragma omp parallel for
	for (Orbitalbody& body : bodies)
	{
		body.UpdateVelocity(dt);
	}
}

void Orbitalsim::GetParents(std::vector<Orbitalbody>& bodies) 
{
	#pragma omp parallel for
	for (int i = 0; i < bodies.size(); i++)
	{
		std::string parentName = "";
		double largestForce = 0.0;

		for (int j = 0; j < bodies.size(); j++)
		{
			if (bodies[j].craft)
			{
				continue;
			}

			if (i != j)
			{
				Vector3d r = bodies[j].position - bodies[i].position;
				double lengthSqr = r.lengthSqr();
				Vector3d tmp = r / (std::max(lengthSqr, 1.1920929E-7) * std::sqrt(lengthSqr));

				Vector3d force = Gkm * tmp;
				double forceStregth = force.length();
				if (forceStregth > largestForce && bodies[j].mass > bodies[i].mass)
				{
					largestForce = forceStregth;
					parentName = bodies[j].name;
				}
			}
		}

		bodies[i].parentName = parentName;
	}
}

Trajectory FindTrajectory(Orbitalsim& sim, const std::string& craftName, const std::string& targetName)
{
	
}

bool RunTrajectory(Orbitalsim& sim, const Trajectory& trajectory)
{
	std::unordered_map<std::string, Orbitalbody> bodiesMap = sim.GetBodiesMap();

	auto it = bodiesMap.find(trajectory.craftName);
	auto it2 = bodiesMap.find(trajectory.targetName);
	if (it == bodiesMap.end() || it2 == bodiesMap.end())
	{
		return false;
	}

	it->second.thrust = trajectory.thrust;
	sim.EditBody(it->second);

	sim.Update(trajectory.duration / 2, 1);

	bodiesMap = sim.GetBodiesMap();
	Orbitalbody craft = bodiesMap[trajectory.craftName]; 
	craft.thrust = -craft.thrust;
	sim.EditBody(craft);

	sim.Update(trajectory.duration / 2, 1);

	return true;
}