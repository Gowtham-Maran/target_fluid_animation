// Copyright (c) 2018 Doyub Kim
//
// I am making my contributions/submissions to this project solely in my
// personal capacity and am not conveying any rights to any intellectual
// property of any third parties.

#include <jet/jet.h>
#include <pystring/pystring.h>

#ifdef JET_WINDOWS
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include <example_utils/clara_utils.h>
#include <clara.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define APP_NAME "target_fluid_animation"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <vector>

#include <random>

using namespace jet;

int fullRangeOfFrames = 95;
int targetSimulationInstances = 1;
int numOfKeyFramesSelected = 1;
bool isFirstPass = true;
std::vector<Vector3D> geometricCentre;

int lowerCouplingFrame = 30;
int upperCouplingFrame = 30;
int blendingParticleSamplingFactor = 2;
int blendingOutlierFrameRange = 8;
int lowerBlendingFrame = lowerCouplingFrame - blendingOutlierFrameRange;
int upperBlendingFrame = upperCouplingFrame + blendingOutlierFrameRange;

Size3 resolutionSource;
BoundingBox3D domainSource;

std::vector<ImplicitTriangleMesh3Ptr> targetShapes;
ImplicitTriangleMesh3Ptr targetShape;
int currentKeyFrame = 0;
Vector3D bbLowerCorner, bbUpperCorner;
BoundingBox3D targetShapeBoundingBox;
double spacingBetweenParticles = 0.01;

double targetSamplingFactor = 0.99;
RigidBodyCollider3Ptr collider;

template<typename Numeric, typename Generator = std::mt19937>
Numeric random(Numeric from, Numeric to)
{
    thread_local static Generator gen(std::random_device{}());

    using dist_type = typename std::conditional
        <
        std::is_integral<Numeric>::value
        , std::uniform_int_distribution<Numeric>
        , std::uniform_real_distribution<Numeric>
        >::type;

    thread_local static dist_type dist;

    return dist(gen, typename dist_type::param_type{ from, to });
}

bool IsValid(Vector3D candidate, double radius, int width, int height,
    int depth, int x_id, int y_id, int z_id,
    std::vector<Vector3D> points, std::vector<int> grid) {
    int searchStartX = std::max(0, x_id - 2);
    int searchEndX = std::min(x_id + 2, width - 1);
    int searchStartY = std::max(0, y_id - 2);
    int searchEndY = std::min(y_id + 2, height - 1);
    int searchStartZ = std::max(0, z_id - 2);
    int searchEndZ = std::min(z_id + 2, depth - 1);

    for (int x = searchStartX; x <= searchEndX; x++) {
        for (int y = searchStartY; y <= searchEndY; y++) {
            for (int z = searchStartZ; z <= searchEndZ; z++) {
                int pointIndex = grid[x + width * (y + height * z)];
                if (pointIndex != -1) {
                    double sqrDst =
                        candidate.distanceSquaredTo(points[pointIndex]);
                    if (sqrDst < radius * radius) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

std::vector<Vector3D> GeneratePoissonDiskSamplingPoints(
    double radius, Vector3D initialPnt, Vector3D sampleRegionSize,
    int numSamplesBeforeRejection = 30) {
    double cellSize = radius / sqrt(3);
    int width = (int)ceil(sampleRegionSize.x / cellSize);
    int height = (int)ceil(sampleRegionSize.y / cellSize);
    int depth = (int)ceil(sampleRegionSize.z / cellSize);
    int gridSize = width * height * depth;

    std::vector<int> grid(gridSize, -1);

    std::vector<Vector3D> points;
    std::vector<Vector3D> spawnPoints;
    srand((unsigned int)time(0));

    spawnPoints.push_back(initialPnt);
    points.push_back(initialPnt);
    Vector3D candidateRange = initialPnt - bbLowerCorner;
    int x_id = (int)(candidateRange.x / cellSize);
    int y_id = (int)(candidateRange.y / cellSize);
    int z_id = (int)(candidateRange.z / cellSize);
    grid[x_id + width * (y_id + height * z_id)] = (int)points.size() - 1;

    while (spawnPoints.size() > 0) {
        int spawnIndex = rand() % spawnPoints.size();
        Vector3D spawnCentre = spawnPoints[spawnIndex];
        bool candidateAccepted = false;

        for (int i = 0; i < numSamplesBeforeRejection; i++) {
            double alpha = ((double)rand() / (RAND_MAX)) * (22.0 / 7.0) * 2;
            double theta = ((double)rand() / (RAND_MAX)) * (22.0 / 7.0) * 2;
            Vector3D dir = Vector3D(sin(alpha) * cos(theta),
                sin(alpha) * sin(theta), cos(alpha));
            Vector3D candidate =
                spawnCentre +
                dir * (1 + ((double)rand() / (RAND_MAX))) * radius;

            double pntSDF = targetShape->signedDistance(candidate);
            if (pntSDF <= 0.0 && pntSDF > -0.02) {
                candidateRange = candidate - bbLowerCorner;
                x_id = (int)(candidateRange.x / cellSize);
                y_id = (int)(candidateRange.y / cellSize);
                z_id = (int)(candidateRange.z / cellSize);
                if (IsValid(candidate, radius, width, height, depth, x_id, y_id,
                    z_id, points, grid)) {
                    points.push_back(candidate);
                    spawnPoints.push_back(candidate);
                    grid[x_id + width * (y_id + height * z_id)] =
                        (int)points.size() - 1;
                    candidateAccepted = true;
                    break;
                }
            }
        }
        if (!candidateAccepted) {
            spawnPoints.erase(spawnPoints.begin() + spawnIndex);
        }
    }

    return points;
}

void saveParticleFromVectorList(
    std::vector<Vector3D> points,
    const std::string& rootDir /*, int time = 0, int numOfSamples = 0*/) {
    char basename[256];
    snprintf(basename, sizeof(basename), "51.txt");
    std::string filename = pystring::os::path::join(rootDir, basename);
    std::ofstream file(filename.c_str(), std::ios::binary);
    if (file) {
        printf("Writing %s...\n", filename.c_str());
        // file << time << ' ' << numOfSamples << ' ' << 0 << std::endl;
        for (int id = 0; id < (int)points.size(); id++) {
            file << points[id].x << ' ' << points[id].y << ' ' << points[id].z
                << std::endl;
        }
        file.close();
    }
}

void saveSourceParticleDetails(const ParticleSystemData3Ptr& particles, int frameCnt, const std::string& srcDir, const std::string& format = ".txt") {

    ArrayAccessor1<Vector3D> positions = particles->positions();

    std::string filename = srcDir + "/" + std::to_string(frameCnt) + format;
    std::ofstream file1(filename.c_str());
    if (file1) {
        printf("Writing %s...\n", filename.c_str());
        for (const auto& pt : positions) {
            file1 << pt.x << ' ' << pt.y << ' ' << pt.z << std::endl;
        }
        file1.close();
    }

    ArrayAccessor1<Vector3D> velocities = particles->velocities();
    filename = srcDir + "/v" + std::to_string(frameCnt) + format;
    std::ofstream file2(filename.c_str());
    if (file2) {
        printf("Writing %s...\n", filename.c_str());
        for (const auto& v : velocities) {
            file2 << v.x << ' ' << v.y << ' ' << v.z << std::endl;
        }
        file2.close();
    }
}

void saveTargetForwardSimulationParticleDetails(const ParticleSystemData3Ptr& particles, int couplingFrame, int currentFrame, const std::string& targetDir, const std::string& format = ".txt") {

    ArrayAccessor1<Vector3D> positions = particles->positions();
    std::string filename = targetDir + "/" + std::to_string(currentFrame) + format;
    std::ofstream file1(filename.c_str(), std::ios_base::app);
    if (file1) {
        printf("Writing %s...\n", filename.c_str());
        for (const auto& pt : positions) {
            file1 << pt.x << ' ' << pt.y << ' ' << pt.z << std::endl;
        }
        file1.close();
    }

    if (currentFrame == couplingFrame) {

        filename = targetDir + "/coupling" + std::to_string(currentFrame) + format;
        std::ofstream file2(filename.c_str());
        if (file2) {
            printf("Writing %s...\n", filename.c_str());
            for (const auto& pt : positions) {
                file2 << pt.x << ' ' << pt.y << ' ' << pt.z << std::endl;
            }
            file2.close();
        }

        ArrayAccessor1<Vector3D> velocities = particles->velocities();
        filename = targetDir + "/couplingV" + std::to_string(currentFrame) + format;
        std::ofstream file3(filename.c_str());
        if (file3) {
            printf("Writing %s...\n", filename.c_str());
            for (const auto& v : velocities) {
                file3 << v.x << ' ' << v.y << ' ' << v.z << std::endl;
            }
            file3.close();
        }
    }
}

void saveTargetBackwardSimulationParticleDetails(const ParticleSystemData3Ptr& particles, int currentFrame, const std::string& targetDir, const std::string& format = ".txt") {

    ArrayAccessor1<Vector3D> positions = particles->positions();

    std::string filename = targetDir + "/" + std::to_string(currentFrame) + format;
    std::ofstream file1(filename.c_str(), std::ios_base::app);
    if (file1) {
        printf("Writing %s...\n", filename.c_str());
        for (const auto& pt : positions) {
            file1 << pt.x << ' ' << pt.y << ' ' << pt.z << std::endl;
        }
        file1.close();
    }
}

void printInfo(const PicSolver3Ptr& solver) {
    auto grids = solver->gridSystemData();
    Size3 resolution = grids->resolution();
    BoundingBox3D domain = grids->boundingBox();
    Vector3D gridSpacing = grids->gridSpacing();

    printf("Resolution: %zu x %zu x %zu\n", resolution.x, resolution.y,
        resolution.z);
    printf("Domain: [%f, %f, %f] x [%f, %f, %f]\n", domain.lowerCorner.x,
        domain.lowerCorner.y, domain.lowerCorner.z, domain.upperCorner.x,
        domain.upperCorner.y, domain.upperCorner.z);
    printf("Grid spacing: [%f, %f, %f]\n", gridSpacing.x, gridSpacing.y,
        gridSpacing.z);
}

void runSourceSimulation(const PicSolver3Ptr& solver, int numberOfFrames, double fps, const std::string& srcDir, const std::string& format = ".txt") {
    auto particles = solver->particleSystemData();
    for (Frame frame(0, 1.0 / fps); frame.index < numberOfFrames; ++frame) {
        solver->update(frame);
        saveSourceParticleDetails(particles, frame.index, srcDir, format);
    }
}

void runTargetForwardSimulation(const PicSolver3Ptr& solver, int couplingFrame, int numberOfFrames, double fps, const std::string& srcDir, const std::string& targetDir, const std::string& format = ".txt") {
    auto particles = solver->particleSystemData();
    for (Frame frame(0, 1.0 / fps); frame.index < numberOfFrames; ++frame) {
        solver->update(frame);
        int currentFrame = couplingFrame + frame.index;
        // Target particles initial velocity selection: Random with shepard method
        if (frame.index == 0) {
            Array1<Vector3D> positionsSolver(particles->numberOfParticles());
            copyRange1(particles->positions(), particles->numberOfParticles(), &positionsSolver);

            Array1<Vector3D> velocitiesSolver(particles->numberOfParticles());
            copyRange1(particles->velocities(), particles->numberOfParticles(), &velocitiesSolver);

            Array1<Vector3D> forcesSolver(particles->numberOfParticles());
            copyRange1(particles->forces(), particles->numberOfParticles(), &forcesSolver);

            particles->resize(0);

            // Signed Distance Field (SDF)
            for (int id = 0; id < (int)positionsSolver.size(); id++) {
                //double signedDist = targetShape->signedDistance(positionsSolver[id]);
                //if (signedDist > -spacingBetweenParticles) { //star_1
                    if (positionsSolver[id].z > 1.4) { //star_2
                    //if (positionsSolver[id].z > 1.59) { //star_3
                    //if (signedDist > -spacingBetweenParticles) { //star_4
                    particles->addParticle(positionsSolver[id], velocitiesSolver[id], forcesSolver[id]);
                }
            }

            ArrayAccessor1<Vector3D> positions = particles->positions();
            ArrayAccessor1<Vector3D> velocities = particles->velocities();

            // Source particle selection
            std::vector<Vector3D> sourcePositions;
            std::vector<Vector3D> sourceVelocities;

            std::string filename = srcDir + "/" + std::to_string(currentFrame) + format;
            std::ifstream file1(filename);
            if (file1.is_open()) {
                std::string line;
                while (std::getline(file1, line)) {
                    std::istringstream iss(line);
                    double x, y, z;
                    if (!(iss >> x >> y >> z)) {
                        break;
                    }  // error

                    sourcePositions.push_back(Vector3D(x, y, z));
                }
                file1.close();
            }

            filename = srcDir + "/v" + std::to_string(currentFrame) + format;
            std::ifstream file2(filename);
            if (file2.is_open()) {
                std::string line;
                while (std::getline(file2, line)) {
                    std::istringstream iss(line);
                    double x, y, z;
                    if (!(iss >> x >> y >> z)) {
                        break;
                    }  // error

                    sourceVelocities.push_back(Vector3D(x, y, z));
                }
                file2.close();
            }

            std::vector<Vector3D> sourcePositionsSelected;
            std::vector<Vector3D> sourceVelocitiesSelected;

			//double targetShapeBBLength = bbUpperCorner.x - bbLowerCorner.x;
            //double targetShapeBBHeight = bbUpperCorner.y - bbLowerCorner.y;
            //double targetShapeBBWidth = bbUpperCorner.z - bbLowerCorner.z;

            //double bbOuterTolerance = 0.5;
            //Vector3D bbOuterUpperCorner = bbUpperCorner + bbOuterTolerance * Vector3D(targetShapeBBLength, targetShapeBBHeight, targetShapeBBWidth);
            //Vector3D bbOuterLowerCorner = bbLowerCorner - bbOuterTolerance * Vector3D(targetShapeBBLength, targetShapeBBHeight, targetShapeBBWidth);
            //BoundingBox3D bbOuter = BoundingBox3D(bbOuterLowerCorner, bbOuterUpperCorner);

            for (int id = 0; id < (int)sourceVelocities.size(); id++) {
                //if(bbOuter.contains(sourcePositions[id])){ //star_1
                if ((sourcePositions[id].y > 0.8) && (sourcePositions[id].x > 3.0)) { //star_2
                //if ((sourcePositions[id].y > 0.6) && (sourcePositions[id].x < 2.0)) { //star_3
                //if ((sourcePositions[id].y > 0.4 && sourcePositions[id].y < 0.8) && (sourcePositions[id].z < 1.5)) { //star_4
                    sourceVelocitiesSelected.push_back(sourceVelocities[id]);
                }
            }

            // Initial velocity assignment
            std::vector<int> targetSamplingParticlesId;
            int targetSamplingSelectedSize = (int)(velocities.size() * targetSamplingFactor);
            int targetSize = (int)velocities.size();
            int sourceVelSelectedSize = (int)sourceVelocitiesSelected.size();
            srand((unsigned int)time(0));
            for (int i = 0; i < targetSamplingSelectedSize; i++) {
                int randTargetId = rand() % targetSize;
                int randSourceId = rand() % sourceVelSelectedSize;
                //star_1
                //velocities[randTargetId] = Vector3D(1.3 * sourceVelocitiesSelected[randSourceId].x, 0/*sourceVelocitiesSelected[randSourceId].y*/, 1.25 * sourceVelocitiesSelected[randSourceId].z);
                //star_2
                velocities[randTargetId] = Vector3D(0.75* sourceVelocitiesSelected[randSourceId].x, 1/*sourceVelocitiesSelected[randSourceId].y*/, 0.75 * sourceVelocitiesSelected[randSourceId].z);
                //star_3
                //velocities[randTargetId] = Vector3D(1.6 * sourceVelocitiesSelected[randSourceId].x, 1/*sourceVelocitiesSelected[randSourceId].y*/, sourceVelocitiesSelected[randSourceId].z);
                // star_4
                //velocities[randTargetId] = Vector3D(0.6 * sourceVelocitiesSelected[randSourceId].x, 0/*sourceVelocitiesSelected[randSourceId].y*/, 1.25 * sourceVelocitiesSelected[randSourceId].z);
                targetSamplingParticlesId.push_back(randTargetId);
            }

            std::sort(targetSamplingParticlesId.begin(), targetSamplingParticlesId.end());

            for (int i = 0; i < (int)positions.size(); i++) {
                Vector3D numerator = Vector3D(0.0, 0.0, 0.0);
                double denominator = 0.0;
                bool notfound = true;
                if (std::find(targetSamplingParticlesId.begin(), targetSamplingParticlesId.end(), i) != targetSamplingParticlesId.end()) {
                    notfound = false;
                }
                else {
                    for (int j = 0; j < (int)targetSamplingParticlesId.size(); j++) {
                        double weight = abs(positions[i].distanceTo(positions[targetSamplingParticlesId[j]]));
                        double weightInv = pow(weight, -4);
                        numerator += weightInv * velocities[targetSamplingParticlesId[j]];
                        denominator += weightInv;
                    }
                }
                if (notfound){
                    //velocities[i] = numerator / denominator;
					Vector3D val = numerator / denominator;
					if(!std::isnan(val.x) && !std::isnan(val.y) && !std::isnan(val.z))
                    	velocities[i] = val;
				}

			}
		}

		//blending
		if (currentFrame > upperBlendingFrame)
        {
            Array1<Vector3D> positionsSolver(particles->numberOfParticles());
            copyRange1(particles->positions(), particles->numberOfParticles(), &positionsSolver);

            Array1<Vector3D> velocitiesSolver(particles->numberOfParticles());
            copyRange1(particles->velocities(), particles->numberOfParticles(), &velocitiesSolver);

            Array1<Vector3D> forcesSolver(particles->numberOfParticles());
            copyRange1(particles->forces(), particles->numberOfParticles(), &forcesSolver);

            particles->resize(0);

            for (int id = 0; id < (int)positionsSolver.size(); id++) {
                if ((positionsSolver[id].y > spacingBetweenParticles) && (collider->surface()->closestDistance(positionsSolver[id]) > spacingBetweenParticles))
                    particles->addParticle(positionsSolver[id], velocitiesSolver[id], forcesSolver[id]);
            }
        }

        if (particles->positions().size() < 2)
            break;

        saveTargetForwardSimulationParticleDetails(particles, couplingFrame, currentFrame, targetDir, format);
    }
}

void runTargetBackwardSimulation(const PicSolver3Ptr& solver, int couplingFrame, int numberOfFrames, double fps, const std::string& srcDir, const std::string& targetDir, const std::string& format = ".txt") {
    auto particles = solver->particleSystemData();
    for (Frame frame(0, 1.0 / fps); frame.index < numberOfFrames; ++frame) {
        solver->update(frame);
        int currentFrame = couplingFrame - frame.index;
        // Target particles initial velocity selection: Random with shepard method
        if (frame.index == 0) {
            particles->resize(0);

            // target forward particle negative velocity assignment
            std::vector<Vector3D> targetForwardPositions;
            std::vector<Vector3D> targetForwardVelocities;

            std::string filename = targetDir + "/coupling" + std::to_string(currentFrame) + format;
			printf("Writing %s...\n", filename.c_str());
            std::ifstream file1(filename);
            if (file1.is_open()) {
                std::string line;
                while (std::getline(file1, line)) {
                    std::istringstream iss(line);
                    double x, y, z;
                    if (!(iss >> x >> y >> z)) {
                        break;
                    }  // error

                    targetForwardPositions.push_back(Vector3D(x, y, z));
                }
                file1.close();
            }
			
            filename = targetDir + "/couplingV" + std::to_string(currentFrame) + format;
            std::ifstream file2(filename);
            if (file2.is_open()) {
                std::string line;
                while (std::getline(file2, line)) {
                    std::istringstream iss(line);
                    double x, y, z;
                    if (!(iss >> x >> y >> z)) {
                        break;
                    }  // error

                    targetForwardVelocities.push_back(Vector3D(x, y, z));
                }
                file2.close();
            }
			printf("targetForwardPositions size %d\n", (int)targetForwardPositions.size() );
			printf("targetForwardVelocities size %d\n", (int)targetForwardVelocities.size() );
            for (int id = 0; id < (int)targetForwardPositions.size(); id++) {
                //Vector3D velocity(-1.25 * targetForwardVelocities[id].x, 0, -targetForwardVelocities[id].z); //star_1
				//particles->addParticle(targetForwardPositions[id], velocity); //star_1
                Vector3D velocity(-0.75 * targetForwardVelocities[id].x, -1, -targetForwardVelocities[id].z); //star_2
				particles->addParticle(targetForwardPositions[id], velocity); //star_2
                //Vector3D velocity(-targetForwardVelocities[id].x, 0, -targetForwardVelocities[id].z); //star_3
                //particles->addParticle(targetForwardPositions[id], -targetForwardVelocities[id]); //star_4
            }
        }

		//blending
		if (currentFrame < lowerBlendingFrame)
        {
            Array1<Vector3D> positionsSolver(particles->numberOfParticles());
            copyRange1(particles->positions(), particles->numberOfParticles(), &positionsSolver);

            Array1<Vector3D> velocitiesSolver(particles->numberOfParticles());
            copyRange1(particles->velocities(), particles->numberOfParticles(), &velocitiesSolver);

            Array1<Vector3D> forcesSolver(particles->numberOfParticles());
            copyRange1(particles->forces(), particles->numberOfParticles(), &forcesSolver);

            particles->resize(0);

            for (int id = 0; id < (int)positionsSolver.size(); id++) {
                if ((positionsSolver[id].y > spacingBetweenParticles) && (collider->surface()->closestDistance(positionsSolver[id]) > spacingBetweenParticles))
                    particles->addParticle(positionsSolver[id], velocitiesSolver[id], forcesSolver[id]);
            }
        }

        if (particles->positions().size() < 2)
            break;
		
        if (currentFrame != couplingFrame)
            saveTargetBackwardSimulationParticleDetails(particles, currentFrame, targetDir, format);
    }
}

void setUpSourceSimulationWaterTank(size_t resolutionX, int numberOfFrames, double fps, const std::string& srcDir, const std::string& format = ".txt") {
    double particleSpace = 0.01;
    double radius = 0.13;
    double domainSizeX = 6;

    // Build solver
    resolutionSource = Size3(6 * resolutionX, 2 * resolutionX, 3 * resolutionX);
    auto solver = FlipSolver3::builder()
        .withResolution(resolutionSource)
        .withDomainSizeX(domainSizeX)
        .makeShared();
    solver->setUseCompressedLinearSystem(true);

    auto grids = solver->gridSystemData();

    domainSource = grids->boundingBox();
    Vector3D bbPlaneLowerCorner(2.25, 0, 1.25);
    Vector3D bbPlaneUpperCorner(2.75, 0.2, 1.75);
    BoundingBox3D bbPlane = BoundingBox3D(bbPlaneLowerCorner, bbPlaneUpperCorner);

    // Build emitter
    auto plane = Plane3::builder()
        .withNormal({ 0, 1, 0 })
        .withPoint({ 0, 0.2, 0 })
        .makeShared();

    auto planeEmitter = VolumeParticleEmitter3::builder()
        .withSurface(plane)
        .withSpacing(particleSpace)
        .withMaxRegion(bbPlane)
        .withIsOneShot(true)
        .makeShared();

    planeEmitter->setPointGenerator(std::make_shared<GridPointGenerator3>());

    auto sphere1 = Sphere3::builder()
        .withCenter(Vector3D(2.25, 1.125, 1.25))
        .withRadius(radius)
        .makeShared();

    auto sphereEmitter1 = VolumeParticleEmitter3::builder()
        .withSurface(sphere1)
        .withSpacing(particleSpace)
        .withMaxRegion(domainSource)
        .withIsOneShot(true)
        .withInitialVelocity(Vector3D(1.51, -6, 1.51))
        .makeShared();
    sphereEmitter1->setPointGenerator(std::make_shared<GridPointGenerator3>());

    auto sphere2 = Sphere3::builder()
        .withCenter(Vector3D(2.25, 1.25, 1.75))
        .withRadius(radius)
        .makeShared();

    auto sphereEmitter2 = VolumeParticleEmitter3::builder()
        .withSurface(sphere2)
        .withSpacing(particleSpace)
        .withMaxRegion(domainSource)
        .withIsOneShot(true)
        .withInitialVelocity(Vector3D(1.38, -6, -1.38))
        .makeShared();
    sphereEmitter2->setPointGenerator(std::make_shared<GridPointGenerator3>());

    auto emitterSet = ParticleEmitterSet3::builder()
        .withEmitters({ planeEmitter, sphereEmitter1, sphereEmitter2 })
        .makeShared();

    solver->setParticleEmitter(emitterSet);

    auto box1 = Box3::builder()
        .withLowerCorner({ -0.025, -0.25, -0.25 })
        .withUpperCorner({ 0.025, 0.25, 0.25 })
        .withOrientation({ {0, 0, 1}, 0.523 })
        .withTranslation({ 2.25, 0.0, 1.5 })
        .makeShared();

    auto box2 = Box3::builder()
        .withLowerCorner({ -0.025, -0.25, -0.25 })
        .withUpperCorner({ 0.025, 0.25, 0.25 })
        .withOrientation({ {0, 0, 1}, -0.523 })
        .withTranslation({ 2.75, 0.0, 1.5 })
        .makeShared();

    auto box3 = Box3::builder()
        .withLowerCorner({ -0.35, -0.2, -0.05 })
        .withUpperCorner({ 0.35, 0.2, 0.05 })
        .withTranslation({ 2.5, 0.0, 1.25 })
        .makeShared();

    auto box4 = Box3::builder()
        .withLowerCorner({ -0.35, -0.2, -0.05 })
        .withUpperCorner({ 0.35, 0.2, 0.05 })
        .withTranslation({ 2.5, 0.0, 1.75 })
        .makeShared();

    auto colliderGeometrySet = ImplicitSurfaceSet3::builder()
        .withExplicitSurfaces({ box1, box2, box3, box4 })
        .makeShared();

    collider = collider = RigidBodyCollider3::builder().withSurface(colliderGeometrySet).makeShared();

    solver->setCollider(collider);

    // Print simulation info
    printf("Running source water tank simulation with FLIP\n");
    printInfo(solver);

    // Run source simulation
    runSourceSimulation(solver, numberOfFrames, fps, srcDir, format);
}

//Target Simulation: Setting Up Solver (FLIP), Emitter, Collider 
void setUpTargetSimulationStar(bool forwardSimulation, size_t resolutionX, int couplingFrame, int numberOfFrames, double fps, const std::string& srcDir, const std::string& targetDir, const std::string& format = ".txt") {
    resolutionSource = Size3(6 * resolutionX, 2 * resolutionX, 3 * resolutionX);
    auto solver = FlipSolver3::builder()
        .withResolution(resolutionSource)
        .withDomainSizeX(6.0)
        .makeShared();
    solver->setUseCompressedLinearSystem(true);

    auto grids = solver->gridSystemData();
    domainSource = grids->boundingBox();

    auto emitter = VolumeParticleEmitter3::builder()
        .withSurface(targetShape)
        .withSpacing(spacingBetweenParticles)
        .withMaxRegion(domainSource)
        .withIsOneShot(true)
        .makeShared();
    emitter->setPointGenerator(std::make_shared<GridPointGenerator3>());

    solver->setParticleEmitter(emitter);

    auto box1 = Box3::builder()
        .withLowerCorner({ -0.025, -0.25, -0.25 })
        .withUpperCorner({ 0.025, 0.25, 0.25 })
        .withOrientation({ {0, 0, 1}, 0.523 })
        .withTranslation({ 2.25, 0.0, 1.5 })
        .makeShared();

    auto box2 = Box3::builder()
        .withLowerCorner({ -0.025, -0.25, -0.25 })
        .withUpperCorner({ 0.025, 0.25, 0.25 })
        .withOrientation({ {0, 0, 1}, -0.523 })
        .withTranslation({ 2.75, 0.0, 1.5 })
        .makeShared();

    auto box3 = Box3::builder()
        .withLowerCorner({ -0.35, -0.2, -0.05 })
        .withUpperCorner({ 0.35, 0.2, 0.05 })
        .withTranslation({ 2.5, 0.0, 1.25 })
        .makeShared();

    auto box4 = Box3::builder()
        .withLowerCorner({ -0.35, -0.2, -0.05 })
        .withUpperCorner({ 0.35, 0.2, 0.05 })
        .withTranslation({ 2.5, 0.0, 1.75 })
        .makeShared();

    auto colliderGeometrySet = ImplicitSurfaceSet3::builder()
        .withExplicitSurfaces({ box1, box2, box3, box4 })
        .makeShared();

    collider = collider = RigidBodyCollider3::builder().withSurface(colliderGeometrySet).makeShared();

    solver->setCollider(collider);

    // Print simulation info
    printf("Running target star simulation with FLIP\n");
    printInfo(solver);

    // Run target simulation
    if (forwardSimulation)
        runTargetForwardSimulation(solver, couplingFrame, numberOfFrames, fps, srcDir, targetDir, format);
    else
        runTargetBackwardSimulation(solver, couplingFrame, numberOfFrames, fps, srcDir, targetDir, format);
}

int main(int argc, char* argv[]) {
    bool showHelp = false;
    size_t resolutionX = 50;
    double fps = 60.0;
    int exampleNum = 1;
    std::string logFilename = APP_NAME ".log";
    std::string outputDirSource = APP_NAME "_source_waterTank";
    std::string outputDirTarget = APP_NAME "_target_star";
    std::string format = ".txt";

    // Parsing
    auto parser =
        clara::Help(showHelp) |
        clara::Opt(resolutionX, "resolutionX")["-r"]["--resx"](
            "grid resolution in x-axis (default is 50)") |
        clara::Opt(fullRangeOfFrames, "fullRangeOfFrames")["-f"]["--frames"](
            "total number of frames (default is 100)") |
        clara::Opt(
            fps, "fps")["-p"]["--fps"]("frames per second (default is 60.0)") |
        clara::Opt(exampleNum, "exampleNum")["-e"]["--example"](
            "example number (between 1 and 6, default is 1)") |
        clara::Opt(logFilename, "logFilename")["-l"]["--log"](
            "log file name (default is " APP_NAME ".log)") |
        clara::Opt(outputDirSource, "outputDirSource")["-o"]["--outputSource"](
            "output directory name (default is " APP_NAME "_source_waterTank)") |
        clara::Opt(outputDirTarget, "outputDirTarget")["-o"]["--outputTarget"](
            "output directory name (default is " APP_NAME "_target_star)") |
        clara::Opt(format, "format")["-m"]["--format"](
            "particle output format (.txt or .xyz or .pos. default is .txt)");

    auto result = parser.parse(clara::Args(argc, argv));
    if (!result) {
        std::cerr << "Error in command line: " << result.errorMessage() << '\n';
        exit(EXIT_FAILURE);
    }

    if (showHelp) {
        std::cout << toString(parser) << '\n';
        exit(EXIT_SUCCESS);
    }

#ifdef JET_WINDOWS
    _mkdir(outputDirSource.c_str());
    _mkdir(outputDirTarget.c_str());
#else
    mkdir(outputDirSource.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
    mkdir(outputDirTarget.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
#endif

    std::ofstream logFile(logFilename.c_str());
    if (logFile) {
        Logging::setAllStream(&logFile);
    }
	
    setUpSourceSimulationWaterTank(50, 96, fps, outputDirSource);

    isFirstPass = false;
    targetSimulationInstances = 1;
    numOfKeyFramesSelected = 1;
    lowerCouplingFrame = 30;
    upperCouplingFrame = 30;
    blendingOutlierFrameRange = 0;
    lowerBlendingFrame = lowerCouplingFrame - blendingOutlierFrameRange;
    upperBlendingFrame = upperCouplingFrame + blendingOutlierFrameRange;

    if (targetSimulationInstances < numOfKeyFramesSelected)
        numOfKeyFramesSelected = targetSimulationInstances;

      for (int keyFrame = 0; keyFrame < numOfKeyFramesSelected; keyFrame++) {
        // Read mesh
        auto mesh = TriangleMesh3::builder().makeShared();
    	std::ifstream objFile(RESOURCES_DIR "/star.obj");
    	if (objFile) {
        	mesh->readObj(&objFile);
		objFile.close();
    	} 
	else {
        	fprintf(stderr, "Cannot open resources/bunny.obj\n");
        	exit(EXIT_FAILURE);
	}			  

        Transform3 transform = Transform3();
        //transform.setOrientation({ {0, 0, 1}, (30.0 / 180.0) * 3.142 }); //star_1
        //transform.setTranslation(Vector3D(3.35, 0.6, 1.9));

        transform.setOrientation({ {0, 0, 1}, (45.0 / 180.0) * 3.142 }); //star_2
        transform.setTranslation(Vector3D(3, 1, 1.4));

        //transform.setOrientation({ {0, 0, 1}, (45.0 / 180.0) * 3.142 }); //star_3
        //transform.setTranslation(Vector3D(1.55, 0.9, 1.6));

        //transform.setOrientation({ {1, 0, 0}, (-45.0 / 180.0) * 3.142 }); //star_4
        //transform.setTranslation(Vector3D(2.6, 0.6, 1.2));

        targetShape = ImplicitTriangleMesh3::builder()
            .withTransform(Transform3(transform))
            .withTriangleMesh(mesh)
            .withResolutionX(50)
            .makeShared();


        targetShapes.push_back(targetShape);
    }

    int couplingInstanceNum = 0;
    for (int couplingFrame = lowerCouplingFrame; couplingFrame <= upperCouplingFrame; couplingFrame++) {
        currentKeyFrame = couplingInstanceNum % numOfKeyFramesSelected;
        targetShape = targetShapes[currentKeyFrame];
        couplingInstanceNum++;

        bbLowerCorner = targetShape->boundingBox().lowerCorner;
        bbUpperCorner = targetShape->boundingBox().upperCorner;
        targetShapeBoundingBox = BoundingBox3D(bbLowerCorner, bbUpperCorner);

		setUpTargetSimulationStar(true, resolutionX, couplingFrame, fullRangeOfFrames - couplingFrame, fps, outputDirSource, outputDirTarget);
        setUpTargetSimulationStar(false, resolutionX, couplingFrame, couplingFrame, fps, outputDirSource, outputDirTarget);

    }

    return EXIT_SUCCESS;
}
