#pragma once

#include "MyVectors.h"

#include <string>
#include <vector>
#include <unordered_map>

struct Orbitalbody
{
	Orbitalbody(const std::string& name = "", const bool craft = false, const double mass = 0, const double radius = 0, const Vector3d position = Vector3dZero(), const Vector3d velocity = Vector3dZero());

	void UpdatePosition(const double dt);
	void UpdateVelocity(const double dt);

	std::string name = "";
	std::string parentName = "";

	bool craft;

	double mass;
	double radius;

	Vector3d position;
	Vector3d velocity;
	Vector3d acceleration, oldAcceleration;
	Vector3d thrust;
};

class Orbitalsim
{
public:

	Orbitalsim(const std::vector<Orbitalbody>& bodies = {});
	~Orbitalsim();

	void Update(const double timeS, const double dt);
	std::pair<std::vector<Orbitalbody>, std::unordered_map<std::string, Orbitalbody>> SoftUpdate(const double timeS, const double dt);

	void SetBodies(const std::vector<Orbitalbody>& bodies);
	std::vector<Orbitalbody> GetBodies();
	void SetBodiesMap(const std::unordered_map<std::string, Orbitalbody>& bodiesMap);
	std::unordered_map<std::string, Orbitalbody> GetBodiesMap();

	bool EditBody(const Orbitalbody& body);

private:

	void Update(std::vector<Orbitalbody>& bodies, const double dt);
	void GetParents(std::vector<Orbitalbody>& bodies);

	std::vector<Orbitalbody> _bodies;
};

struct Trajectory
{
	std::string craftName = "";
	std::string targetName = "";

	Vector3d thrust;
	double duration;
};

Trajectory FindTrajectory(Orbitalsim& sim, const std::string& craftName, const std::string& targetName);
bool RunTrajectory(Orbitalsim& sim, const Trajectory& trajectory);