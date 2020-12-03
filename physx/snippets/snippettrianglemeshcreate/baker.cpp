
#include <ctype.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <thread>
#include <cstdio>
#include <mutex>

#include "PxPhysicsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "svpng.h"

using namespace std;
using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;
PxCooking*				gCooking	= NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene		= NULL;

PxMaterial*				gMaterial	= NULL;

PxPvd*                  gPvd        = NULL;

float					minX = 1000000.0f;
float					maxX = -1000000.0f;
float					minZ = 1000000.0f;
float					maxZ = -1000000.0f;

// Setup common cooking params
void setupCommonCookingParams(PxCookingParams& params, bool skipMeshCleanup, bool skipEdgeData)
{
		// we suppress the triangle mesh remap table computation to gain some speed, as we will not need it 
	// in this snippet
	params.suppressTriangleMeshRemapTable = true;

	// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. The input mesh must be valid. 
	// The following conditions are true for a valid triangle mesh :
	//  1. There are no duplicate vertices(within specified vertexWeldTolerance.See PxCookingParams::meshWeldTolerance)
	//  2. There are no large triangles(within specified PxTolerancesScale.)
	// It is recommended to run a separate validation check in debug/checked builds, see below.

	if (!skipMeshCleanup)
		params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH);
	else
		params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

	// If DISABLE_ACTIVE_EDGES_PREDOCOMPUTE is set, the cooking does not compute the active (convex) edges, and instead 
	// marks all edges as active. This makes cooking faster but can slow down contact generation. This flag may change 
	// the collision behavior, as all edges of the triangle mesh will now be considered active.
	if (!skipEdgeData)
		params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE);
	else
		params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
}

// Creates a triangle mesh using BVH33 midphase with different settings.
PxTriangleMesh* createBV33TriangleMesh(PxU32 numVertices, const PxVec3* vertices, PxU32 numTriangles, const PxU32* indices, 
	bool skipMeshCleanup, bool skipEdgeData, bool inserted, bool cookingPerformance, bool meshSizePerfTradeoff)
{
	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = numVertices;
	meshDesc.points.data = vertices;
	meshDesc.points.stride = sizeof(PxVec3);
	meshDesc.triangles.count = numTriangles;
	meshDesc.triangles.data = indices;
	meshDesc.triangles.stride = 3 * sizeof(PxU32);

	PxCookingParams params = gCooking->getParams();

	// Create BVH33 midphase
	params.midphaseDesc = PxMeshMidPhase::eBVH33;

	// setup common cooking params
	setupCommonCookingParams(params, skipMeshCleanup, skipEdgeData);

	// The COOKING_PERFORMANCE flag for BVH33 midphase enables a fast cooking path at the expense of somewhat lower quality BVH construction.	
	if (cookingPerformance)
		params.midphaseDesc.mBVH33Desc.meshCookingHint = PxMeshCookingHint::eCOOKING_PERFORMANCE;
	else
		params.midphaseDesc.mBVH33Desc.meshCookingHint = PxMeshCookingHint::eSIM_PERFORMANCE;

	// If meshSizePerfTradeoff is set to true, smaller mesh cooked mesh is produced. The mesh size/performance trade-off
	// is controlled by setting the meshSizePerformanceTradeOff from 0.0f (smaller mesh) to 1.0f (larger mesh).
	if(meshSizePerfTradeoff)
	{
		params.midphaseDesc.mBVH33Desc.meshSizePerformanceTradeOff = 0.0f;
	}
	else
	{
		// using the default value
		params.midphaseDesc.mBVH33Desc.meshSizePerformanceTradeOff = 0.55f;
	}

	gCooking->setParams(params);

#if defined(PX_CHECKED) || defined(PX_DEBUG)
	// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. 
	// We should check the validity of provided triangles in debug/checked builds though.
	if (skipMeshCleanup)
	{
		PX_ASSERT(gCooking->validateTriangleMesh(meshDesc));
	}
#endif // DEBUG


	PxTriangleMesh* triMesh = NULL;
	PxU32 meshSize = 0;

	// The cooked mesh may either be saved to a stream for later loading, or inserted directly into PxPhysics.
	if (inserted)
	{
		triMesh = gCooking->createTriangleMesh(meshDesc, gPhysics->getPhysicsInsertionCallback());
	}
	else
	{
		PxDefaultMemoryOutputStream outBuffer;
		gCooking->cookTriangleMesh(meshDesc, outBuffer);

		PxDefaultMemoryInputData stream(outBuffer.getData(), outBuffer.getSize());
		triMesh = gPhysics->createTriangleMesh(stream);

		meshSize = outBuffer.getSize();
	}

	// Print the elapsed time for comparison
	//triMesh->release();
	return triMesh;
}

bool loadMesh(const char* file, int filterIndex)
{
	vector<PxVec3> vertVec;
	vector<PxU32> triVec;
	ifstream inFile;
	inFile.open(file, ios::in);
	if (!inFile.is_open()) return false;
	while (true)
	{
		char type[10];
		inFile >> type;
		if (inFile.eof()) break;
		if (strcmp(type, "v") == 0 || strcmp(type, "V") == 0)
		{
			float x, y, z;
			inFile >> x >> y >> z;
			//x -= 5000.0f;
			//z -= 5000.0f;
			vertVec.emplace_back(x, y, z);

			if (x < minX) minX = x;
			if (x > maxX) maxX = x;
			if (z < minZ) minZ = z;
			if (z > maxZ) maxZ = z;
		}
		else if (strcmp(type, "f") == 0 || strcmp(type, "F") == 0)
		{
			int v1, v2, v3;
			inFile >> v1 >> v2 >> v3;
			triVec.emplace_back(v1 - 1);
			triVec.emplace_back(v2 - 1);
			triVec.emplace_back(v3 - 1);
		}
		else
		{
			fprintf(stderr, "wrong obj file: %s\n", file);
			return false;
		}
	}
	inFile.close();

	PxTriangleMesh* mesh = createBV33TriangleMesh((PxU32)vertVec.size(), reinterpret_cast<const PxVec3*>(&vertVec[0]), (PxU32)triVec.size() / 3, reinterpret_cast<const PxU32*>(&triVec[0]), false, false, false, false, false);
	if (mesh)
	{
		PxShape* shape = gPhysics->createShape(PxTriangleMeshGeometry(mesh), *gMaterial);
		PxFilterData filterData;
		filterData.word0 = (1 << filterIndex);
		//shape->setSimulationFilterData(filterData);
		shape->setQueryFilterData(filterData);
		PxRigidStatic* actor = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0, 0, 0)), *shape);
		gScene->addActor(*actor);
		shape->release();
		mesh->release();
	}

	return true;
}

void loadMeshs(const char *dir)
{
	char path[4096];
	for (int i = 1; i < 10000; i++)
	{
		snprintf(path, sizeof(path) - 1, "%s\\%d.obj", dir, i);
		if (!loadMesh(path, 0)) break;
	}
}

void initPhysics()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	//gDispatcher = PxDefaultCpuDispatcherCreate(2);
	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);
}

void stepPhysics()
{
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}

void cleanupPhysics()
{
	if (gScene) gScene->release();
	if (gDispatcher) gDispatcher->release();
	if (gCooking) gCooking->release();
	if (gPhysics) gPhysics->release();
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		if (transport) transport->release();
	}
	if (gFoundation) gFoundation->release();
}

void bake2png(int width, int height, const PxVec3 *data)
{
#if 1
	const int blockWidth = 2048;
	const int blockHeight = 2048;

	if ((width % blockWidth) != 0) return;
	if ((height % blockHeight) != 0) return;

	unsigned char* rgb = new unsigned char[blockWidth * blockHeight * 3];
	for (int w = 0; w < width / blockWidth; w++)
	{
		for (int h = 0; h < height / blockHeight; h++)
		{
			unsigned char* p = rgb;
			int x, z;
			char path[1024];
			snprintf(path, sizeof(path) - 1, "d:\\ForceFields\\%02d_%02d.png", height / blockHeight - 1 - h, w);
			FILE* fp = fopen(path, "wb");
			//for (z = 0; z < blockHeight; z++)
			for (z = blockHeight-1; z >= 0; z--)
			{
				for (x = 0; x < blockWidth; x++) {
					const PxVec3& v = data[(h * blockHeight + z) * width + (w * blockWidth + x)];
					float m = v.magnitude();
					m = pow(pow(m, 1.0f / 3) / 10.0f, 1.0f / 3);
					if (m < 0.001f)
					{
						m = 0.001f;
					}
					else if (m > 0.999f)
					{
						m = 0.999f;
					}
					*p++ = (unsigned char)(256 * m); //R
					*p++ = 0; //G
					*p++ = 0; //B
				}
			}
			svpng(fp, blockWidth, blockHeight, rgb, 0);
			fclose(fp);
		}
	}
	delete[] rgb;
#else
	unsigned char* rgb = new unsigned char[width * height * 3];
	unsigned char* p = rgb;
    int x, z;
    FILE *fp = fopen("d:\\rgb.png", "wb");
	for (z = 0; z < height; z++)
	{
		for (x = 0; x < width; x++) {
			const PxVec3& v = data[z * width + x];
			float m = v.magnitude();
			m = pow(pow(m, 1.0f / 3) / 10.0f, 1.0f / 3);
			if (m < 0.001f)
			{
				m = 0.001f;
			}
			else if (m > 0.999f)
			{
				m = 0.999f;
			}
			*p++ = (unsigned char)(256 * m); //R
			*p++ = 0; //G
			*p++ = 0; //B
		}
	}
    svpng(fp, width, height, rgb, 0);
    fclose(fp);
	delete[] rgb;
#endif
}

void bake(const char *file)
{
	//std::cout << minX << endl;
	//std::cout << maxX << endl;
	//std::cout << minZ << endl;
	//std::cout << maxZ << endl;

	const int texWidth = 4096*8;
	const int texHeight = 2048*8;
	const int blockWidth = 1024;
	const int blockHeight = 1024;
	const float coreRange = 0.3f;
	const float softRange = 1.0f;
	const float forceMax = 1000.0f;

	PxVec3* forceFields = new PxVec3[texWidth * texHeight];
	for (int i = 0; i < texWidth * texHeight; i++)
	{
		forceFields[i] = PxVec3(0.0f);
	}

	PxU64 startTime = SnippetUtils::getCurrentTimeCounterValue();

	const float xStep = (maxX - minX + 2.0f) / texWidth;
	const float zStep = (maxZ - minZ + 2.0f) / texHeight;
	const float xBegin = minX - 1.0f;
	const float zBegin = minZ - 1.0f;

#if 1
	const int threadCount = 16;
	std::thread t[threadCount];
	int count[threadCount];
	for (int ti = 0; ti < threadCount; ti++)
	{
		t[ti] = std::thread([=, &count] {
			int blockIndex = 0;
			count[ti] = 0;
			for (int xMin = 0, xMax = blockWidth; xMin < texWidth; xMin += blockWidth, xMax += blockWidth)
			{
				for (int zMin = 0, zMax = blockHeight; zMin < texHeight; zMin += blockHeight, zMax += blockHeight)
				{
					if ((blockIndex++ % threadCount) != ti) continue;
					count[ti]++;
					for (int x = xMin; x < texWidth && x < xMax; x++)
					{
						for (int z = zMin; z < texHeight && z < zMax; z++)
						{
							PxVec3 from(xBegin + x * xStep, 2.0f, zBegin + z * zStep);
#if 1
							PxOverlapBuffer hit;
							if (gScene->overlap(PxSphereGeometry(1.0f), PxTransform(from), hit, PxQueryFilterData(PxQueryFlag::eANY_HIT | PxQueryFlag::eSTATIC)))
							{
								PxVec3 force(0.0f);
								for (int i = 0; i < 36; i++)
								{
									PxQuat qStep(i * 10.0f / 180.0f * 3.14159f, PxVec3(0, 1, 0));
									PxVec3 dir(0, 0, 1);
									dir = qStep.rotate(dir);
									PxRaycastBuffer hit2;
									if (gScene->raycast(from, dir, 1.1f, hit2))
									{
										if (!hit2.hasBlock) throw 1; //FIXME:
										PxVec3 f = from;
										f -= hit2.block.position;
										float m = f.normalize();
										m = (m - coreRange) / (softRange - coreRange);
										if (m < 0.001f)
										{
											m = 0.001f;
										}
										else if (m > 0.999f)
										{
											m = 0.999f;
										}
										float factor = 1 / m - 1;
										force += f * factor;
									}
								}
								float mag = force.normalize();
								if (mag > 0.001f)
								{
									if (mag > forceMax) mag = forceMax;
								}
								forceFields[z * texWidth + x] = force * mag; //FIXME: 多线程cache问题？
							}
#else
							forceFields[z * texWidth + x] = from;
#endif
						}
					}
				}
			}
		});
	}
	for (int ti = 0; ti < threadCount; ti++)
	{
		t[ti].join();
	}
#else
	for (int x = 0; x < texWidth; x++)
	{
		for (int z = 0; z < texHeight; z++)
		{
			PxVec3 from(xBegin + x * xStep, 2.0f, zBegin + z * zStep);
#if 1
			PxOverlapBuffer hit;
			if (gScene->overlap(PxSphereGeometry(1.0f), PxTransform(from), hit, PxQueryFilterData(PxQueryFlag::eANY_HIT | PxQueryFlag::eSTATIC)))
			{
				PxVec3 force(0.0f);
				for (int i = 0; i < 36; i++)
				{
					PxQuat qStep(i * 10.0f / 180.0f * 3.14159f, PxVec3(0, 1, 0));
					PxVec3 dir(0, 0, 1);
					dir = qStep.rotate(dir);
					PxRaycastBuffer hit2;
					if (gScene->raycast(from, dir, 1.1f, hit2))
					{
						if (!hit2.hasBlock) throw 1; //FIXME:
						PxVec3 f = from;
						f -= hit2.block.position;
						float m = f.normalize();
						m = (m - coreRange) / (softRange - coreRange);
						if (m < 0.001f)
						{
							m = 0.001f;
						}
						else if (m > 0.999f)
						{
							m = 0.999f;
						}
						float factor = 1 / m - 1;
						force += f * factor;
					}
				}
				float mag = force.normalize();
				if (mag > 0.001f)
				{
					if (mag > forceMax) mag = forceMax;
				}
				forceFields[z * texWidth + x] = force * mag;
			}
#else
			forceFields[z * texWidth + x] = from;
#endif
		}
	}
#endif

	PxU64 stopTime = SnippetUtils::getCurrentTimeCounterValue();

	FILE* fp = fopen(file, "w");
	if (!fp) return;
	fprintf(fp, "%f %f %f %f %d %d\n", xBegin, xStep, zBegin, zStep, texWidth, texHeight);
#if 0
	for (int i = 0; i < threadCount; i++)
	{
		fprintf(fp, "%d %d\n", i, count[i]);
	}
#endif
	for (int i = 0; i < texWidth * texHeight; i++)
	{
		if (forceFields[i].magnitudeSquared() > 0.001f)
		{
			int x = i % texWidth;
			int z = i / texWidth;
			fprintf(fp, "%d %d %0.2f %0.2f\n", x, z, forceFields[i].x, forceFields[i].z);
		}
	}
	fclose(fp);

	float elapsedTime = SnippetUtils::getElapsedTimeInMilliseconds(stopTime - startTime);
	printf("#Elapsed time in ms: %f \n", double(elapsedTime));

	bake2png(texWidth, texHeight, forceFields);

	delete forceFields;
	forceFields = nullptr;
}

std::mutex output_mutex;
void bake2(const char *file)
{
	//const int texWidth = 4096*8;
	//const int texHeight = 4096*8;
	//const int texWidth = 1024*8;
	//const int texHeight = 1024*8;
	int w = int(maxX - minX);
	w = (w + 1023) / 1024 * 1024;
	int texWidth = w * 8;
	int h = int(maxZ - minZ);
	h = (h + 1023) / 1024 * 1024;
	int texHeight = h * 8;

	const int blockWidth = 1024;
	const int blockHeight = 1024;
	const float coreRange = 0.3f;
	const float softRange = 1.0f;
	const float forceMax = 1000.0f;

	PxU64 startTime = SnippetUtils::getCurrentTimeCounterValue();

	const float xStep = (maxX - minX + 2.0f) / texWidth;
	const float zStep = (maxZ - minZ + 2.0f) / texHeight;
	const float xBegin = minX - 1.0f;
	const float zBegin = minZ - 1.0f;

	FILE* fp = fopen(file, "w");
	if (!fp) return;
	fprintf(fp, "%f %f %f %f %d %d\n", xBegin, xStep, zBegin, zStep, texWidth, texHeight);

	const int threadCount = 16;
	std::thread t[threadCount];
	volatile int count[threadCount];
	for (int ti = 0; ti < threadCount; ti++)
	{
		t[ti] = std::thread([=, &count] {
			std::vector<std::string> results;
			int blockIndex = 0;
			count[ti] = 0;
			for (int xMin = 0, xMax = blockWidth; xMin < texWidth; xMin += blockWidth, xMax += blockWidth)
			{
				for (int zMin = 0, zMax = blockHeight; zMin < texHeight; zMin += blockHeight, zMax += blockHeight)
				{
					if ((blockIndex++ % threadCount) != ti) continue;
					count[ti]++;
					for (int x = xMin; x < texWidth && x < xMax; x++)
					{
						for (int z = zMin; z < texHeight && z < zMax; z++)
						{
							PxVec3 rayFrom(xBegin + x * xStep, -10000, zBegin + z * zStep);
							while (true)
							{
								PxVec3 rayDir(0, 1, 0);
								PxRaycastBuffer rayHit;
								PxQueryFilterData rayFilterData;
								rayFilterData.data.word0 = 1;
								if (!gScene->raycast(rayFrom, rayDir, 20000, rayHit, PxHitFlags(PxHitFlag::eDEFAULT), rayFilterData))
								{
									break;
								}
								if (!rayHit.hasBlock) throw 1; //FIXME:

								PxVec3 from = rayHit.block.position;
								PxVec3 force(0.0f);
								for (int i = 0; i < 36; i++)
								{
									PxQuat qStep(i * 10.0f / 180.0f * 3.14159f, PxVec3(0, 1, 0));
									PxVec3 dir(0, 0, 1);
									dir = qStep.rotate(dir);
									PxRaycastBuffer hit;
									PxQueryFilterData filterData;
									filterData.data.word0 = 2;
									if (gScene->raycast(from, dir, 1.1f, hit, PxHitFlags(PxHitFlag::eDEFAULT), filterData))
									{
										if (!hit.hasBlock) throw 2; //FIXME:
										PxVec3 f = from;
										f -= hit.block.position;
										f.y = 0.0f;
										float m = f.normalize();
										m = (m - coreRange) / (softRange - coreRange);
										if (m < 0.001f)
										{
											m = 0.001f;
										}
										else if (m > 0.999f)
										{
											m = 0.999f;
										}
										float factor = 1 / m - 1;
										force += f * factor;
									}
								}
								float mag = force.normalize();
								if (mag > 0.01f) //FIXME:
								{
									if (mag > forceMax) mag = forceMax;
									force *= mag;
									char buf[200];
									sprintf(buf, "%d %d %0.2f %0.2f %0.2f", x, z, from.y, force.x, force.z);
									results.push_back(buf);
									if (results.size() > 1000)
									{
										//lock and output
										std::lock_guard<std::mutex> lock(output_mutex);
										for (auto o : results)
										{
											fprintf(fp, "%s\n", o.c_str());
										}
										results.clear();
									}
								}
								rayFrom.y = from.y + 1;
							}
						}
					}
				}
			}
			if (results.size() > 0)
			{
				//lock and output
				std::lock_guard<std::mutex> lock(output_mutex);
				for (auto o : results)
				{
					fprintf(fp, "%s\n", o.c_str());
				}
				results.clear();
			}
		});
	}
	for (int ti = 0; ti < threadCount; ti++)
	{
		t[ti].join();
	}

#if 0
	for (int i = 0; i < threadCount; i++)
	{
		fprintf(fp, "%d %d\n", i, count[i]);
	}
#endif
	fclose(fp);

	PxU64 stopTime = SnippetUtils::getCurrentTimeCounterValue();
	float elapsedTime = SnippetUtils::getElapsedTimeInMilliseconds(stopTime - startTime);
	printf("#Elapsed time in ms: %f \n", double(elapsedTime));
}

int snippetMain(int, const char*const*)
{	
	static const PxU32 frameCount = 100;
	initPhysics();

	//loadMesh("D:\\Scene_\\Public\\scene\\windows\\navmesh_scene.test.obj");
	//loadMesh("D:\\Scene_\\Public\\scene\\windows\\navmesh_scene.demo.obj");
	//loadMesh("D:\\Scene_\\Public\\scene\\windows\\navmesh_scene.mainland.floor.obj", 0);
	//loadMesh("D:\\Scene_\\Public\\scene\\windows\\navmesh_scene.mainland.wall.obj", 1);
	loadMesh("D:/work/gitlab/export_scene_data_for_server/windows/ForceFields/data/mainland/navmesh_scene.floor.obj", 0);

	//原始场景
	minX = 1000000.0f;
	maxX = -1000000.0f;
	minZ = 1000000.0f;
	maxZ = -1000000.0f;
	loadMesh("D:/work/gitlab/export_scene_data_for_server/windows/ForceFields/data/mainland/navmesh_scene.wall.obj", 1);

	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics();

	//bake("d:\\ForceFields\\forcefields.txt");
	//bake("D:\\Scene_\\Public\\scene\\windows\\forcefields.test.txt");
	//bake("D:\\Scene_\\Public\\scene\\windows\\forcefields.demo.txt");
	//bake2("D:\\Scene_\\Public\\scene\\windows\\forcefields.mainland.txt");
	bake2("D:/work/gitlab/export_scene_data_for_server/windows/ForceFields/data/mainland/forcefields.txt");

	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics();

	cleanupPhysics();
	return 0;
}
