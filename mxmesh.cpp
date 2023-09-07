////////////////  Developed By Hamid.Memar (2023-Revised)  ////////////////
///////////////   Licensed Under MIT Terms And Arguments   ////////////////


#define _CRT_SECURE_NO_WARNINGS

// MaxSDK
#include <maxscript/maxscript.h>
#include <maxscript/macros/define_instantiation_functions.h>
#include <maxapi.h>
#include <CoreFunctions.h>
#include <inode.h>
#include <polyobj.h>
#include <triobj.h>
#include <mnmesh.h>
#include <mesh.h>
#include <MeshNormalSpec.h>
#include <object.h>
#include <geom\trig.h>
#include <decomp.h> 
#include <control.h>
#include <hold.h>
#include <HoldBegin.h>

// Windows SDK
#include <Windows.h>
#include <filesystem>
#include <fstream>
#include <ppl.h>
#include <sstream>

// Timestamp
#include <chrono>
#include <winsock.h>

// Zipper/Unzipper
#include "mxm_zipper\zipper.h"
#include "mxm_zipper\unzipper.h"
#pragma comment(lib,"mxm_fzlib.lib")
#pragma comment(lib,"mxm_zipper.lib")

// Stopwatch
#include "mxm_stopwatch.h"

// Namespaces
using namespace std;
using namespace zipper;
using namespace concurrency;
using namespace PerformanceTools;
using namespace filesystem;

// Pre-Defined Macros
#define PLUGIN_DESCRIPTION		L"A Fast Mesh Cacher & Checkpoint Tool for 3ds Max, Developed by MemarDesign™ LLC."
#define PLUGIN_CLASS_ID			Class_ID(0x666A31C3, 0x4B5778E)
#define PLUGIN_MXS_STRUCT		"MXMesh"

// Utility Macros
#define MaxMeshMXS(fn, name) Value* fn##_api(Value**,int); Primitive fn##_pf (_M(name), _M(PLUGIN_MXS_STRUCT), fn##_api)
#define SINGLE_THREAD_LOOP_BEGIN(lsize)	for (size_t i = 0; i < lsize; i++) {
#define MULTI_THREAD_LOOP_BEGIN(lsize) parallel_for(size_t(0), (size_t)lsize, [&](size_t i) {
#define SINGLE_THREAD_LOOP_END }
#define MULTI_THREAD_LOOP_END });
#define BUFFER std::vector<unsigned char>
#define BUFFER_FREE(buffer) BUFFER().swap(buffer)

// Logger Macros
#define DebugLog(fmt,...) if(DebugMode) { mprintf(L"[MXMesh] : " fmt L"\n",__VA_ARGS__); }

// Option Macros
#define DISK_CACHE_BUFFERING_MODE					0xF0
#define MEMORY_CACHE_BUFFERING_MODE					0xED
#define RESTORE_CACHE_MODE_SINGLE_THREAD			0xCA
#define RESTORE_CACHE_MODE_MULTI_THREAD				0xBE

// Global Instances
HINSTANCE			hInstance;
Interface*			maxInterface;
Stopwatch			profiler;
fstream				fileWritter;

// Global Values
Class_ID			triobjectCID		(TRIOBJ_CLASS_ID, 0);
string				cachePath			= "C:\\Users\\Public";
Zipper::zipFlags	compressionMode		= Zipper::Better;
BYTE				cacheBufferingMode	= MEMORY_CACHE_BUFFERING_MODE;
BYTE				restoreMode			= RESTORE_CACHE_MODE_MULTI_THREAD;
bool				DebugMode			= false;

// Global Buffers
stringstream vtxBuffer, nrmBuffer, texBuffer, idxBuffer, tdxBuffer, ndxBuffer, mtaBuffer;

// Structures
struct MaxMeshMetaData
{
	int vNum, nNum, tNum, fNum;
	char name[128];
	Point3 pos, rot, scale;
	UINT16 flags;
	AffineParts affine;
	Matrix3 tm;
	DWORD col;
};

// Timestamp Utilities
static int gtoftd(struct timeval* tp, struct timezone* tzp) {
	namespace sc = std::chrono;
	sc::system_clock::duration d = sc::system_clock::now().time_since_epoch();
	sc::seconds s = sc::duration_cast<sc::seconds>(d);
	tp->tv_sec = s.count();
	tp->tv_usec = sc::duration_cast<sc::microseconds>(d - s).count();
	return 0;
}
static char* gtfrmtt() {
	static char buffer[26];

	// For Miliseconds
	int millisec;
	struct tm* tm_info;
	struct timeval tv;

	// For Time
	time_t rawtime;
	struct tm* timeinfo;

	gtoftd(&tv, NULL);

	millisec = lrint(tv.tv_usec / 1000.0);
	if (millisec >= 1000)
	{
		millisec -= 1000;
		tv.tv_sec++;
	}

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, 26, "%Y-%m-%d-%H-%M-%S", timeinfo);
	sprintf_s(buffer, 26, "%s.%03d", buffer, millisec);

	return buffer;
}

// String Utilities
void String2Lower(string& strin) 
{
	std::for_each(strin.begin(), strin.end(), [](char& c) {c = ::tolower(c); });
}
const wchar_t* StringGetWideChar(const char* c)
{
	const size_t cSize = strlen(c) + 1;
	wchar_t* wc = new wchar_t[cSize];
	mbstowcs(wc, c, cSize);

	return wc;
}

// Forward Definitions
bool GenerateNewPolyFromCache(const wchar_t* mxm_package, INode* node);

// Undo/Redo System
class RestoreMeshOp : public RestoreObj
{
public:
	RestoreMeshOp(PolyObject* poly, const wchar_t* mxo_package, INode* node)
	{
		obj = poly;
		this->undo_mnMesh = MNMesh(poly->mm);
		redo_mxo_package = mxo_package;
		redo_node = node;
	}
	~RestoreMeshOp()
	{
		undo_mnMesh.ClearAndFree();
	}
	void Restore(int isUndo)
	{
		obj->mm.ClearAndFree();
		obj->mm = this->undo_mnMesh;
		obj->NotifyDependents(FOREVER, ALL_CHANNELS, REFMSG_CHANGE);
	}
	void Redo()
	{
		GenerateNewPolyFromCache(redo_mxo_package, redo_node);
	}
private:
	PolyObject*		obj;
	MNMesh			undo_mnMesh;
	const wchar_t*	redo_mxo_package;
	INode*			redo_node;
};

// Operations
INode* CreateObjectInScene(SClass_ID sid, Class_ID cid)
{
	Object* obj = static_cast<Object*>(CreateInstance(sid, cid));
	return maxInterface->CreateObjectNode(obj);
}
bool CacheMeshToDisk(INode* node, bool checkpoint = false)
{
	char outputNameBuffer[MAX_PATH];

	if (!node) { return false; }
	DebugLog(L"Caching object [%s] mesh buffer...", node->GetName());

	if (cacheBufferingMode == DISK_CACHE_BUFFERING_MODE)
		DebugLog(L"Config `Cache Buffering Mode` = DISK_CACHE_BUFFERING_MODE");
	if (cacheBufferingMode == MEMORY_CACHE_BUFFERING_MODE)
		DebugLog(L"Config `Cache Buffering Mode` = MEMORY_CACHE_BUFFERING_MODE");

	profiler.Reset(); profiler.Start();

	TimeValue t = GetCOREInterface()->GetTime();
	Object* obj = node->EvalWorldState(t).obj;

	if (obj->CanConvertToType(triobjectCID))
	{
		// Get Tri Object
		TriObject* tobj = (TriObject*)obj->ConvertToType(t, triobjectCID); if (!tobj) { return false; }

		// Get Mesh
		Mesh mesh = tobj->GetMesh();

		// Compute Normals
		mesh.SpecifyNormals();
		MeshNormalSpec* mesh_ns = mesh.GetSpecifiedNormals();
		mesh_ns->CheckNormals();

		// Get Mesh Data Sizes
		int vNum = mesh.numVerts;
		int nNum = mesh_ns->GetNumNormals();
		int fNum = mesh.numFaces;
		int tNum = mesh.numTVerts;

		MaxMeshMetaData meshMeta;
		meshMeta.vNum = vNum;
		meshMeta.nNum = nNum;
		meshMeta.fNum = fNum;
		meshMeta.tNum = tNum;

		// Dump Information
		sprintf_s(meshMeta.name, sizeof meshMeta.name, "%S", node->GetName());
		meshMeta.col = node->GetWireColor();

		// Dump Transformations
		meshMeta.tm = node->GetObjTMAfterWSM(t);
		decomp_affine(meshMeta.tm, &meshMeta.affine);
		QuatToEuler(meshMeta.affine.q, meshMeta.rot);
		meshMeta.pos.x = meshMeta.affine.t.x;
		meshMeta.pos.y = meshMeta.affine.t.y;
		meshMeta.pos.z = meshMeta.affine.t.z;
		meshMeta.rot.x = RadToDeg_float(meshMeta.rot.x);
		meshMeta.rot.y = RadToDeg_float(meshMeta.rot.y);
		meshMeta.rot.z = RadToDeg_float(meshMeta.rot.z);
		meshMeta.scale.x = meshMeta.affine.k.x;
		meshMeta.scale.y = meshMeta.affine.k.y;
		meshMeta.scale.z = meshMeta.affine.k.z;

		// Get Temp Path
		string tempAddr = filesystem::temp_directory_path().string();

		// Dump Raw Buffers
		if (cacheBufferingMode == DISK_CACHE_BUFFERING_MODE)
		{
			fileWritter.open(tempAddr + "\\max-mesh.vtx", ios::binary | ios::out);
			fileWritter.write((char*)mesh.verts, vNum * sizeof(Point3));
			fileWritter.close();

			fileWritter.open(tempAddr + "\\max-mesh.tex", ios::binary | ios::out);
			fileWritter.write((char*)mesh.tVerts, tNum * sizeof(UVVert));
			fileWritter.close();

			fileWritter.open(tempAddr + "\\max-mesh.idx", ios::binary | ios::out);
			fileWritter.write((char*)mesh.faces, fNum * sizeof(Face));
			fileWritter.close();

			fileWritter.open(tempAddr + "\\max-mesh.tdx", ios::binary | ios::out);
			fileWritter.write((char*)mesh.tvFace, fNum * sizeof(TVFace));
			fileWritter.close();

			fileWritter.open(tempAddr + "\\max-mesh.ndx", ios::binary | ios::out);
			fileWritter.write((char*)mesh_ns->GetFaceArray(), fNum * sizeof(MeshNormalFace));
			fileWritter.close();

			fileWritter.open(tempAddr + "\\max-mesh.nrm", ios::binary | ios::out);
			fileWritter.write((char*)mesh_ns->GetNormalArray(), nNum * sizeof(Point3));
			fileWritter.close();

			fileWritter.open(tempAddr + "\\max-mesh.mta", ios::binary | ios::out);
			fileWritter.write((char*)&meshMeta, sizeof(MaxMeshMetaData));
			fileWritter.close();
		}
		if (cacheBufferingMode == MEMORY_CACHE_BUFFERING_MODE)
		{
			vtxBuffer = stringstream(string(reinterpret_cast<char*>((char*)mesh.verts), vNum * sizeof(Point3)));
			nrmBuffer = stringstream(string(reinterpret_cast<char*>((char*)mesh_ns->GetNormalArray()), nNum * sizeof(Point3)));
			texBuffer = stringstream(string(reinterpret_cast<char*>((char*)mesh.tVerts), tNum * sizeof(UVVert)));
			idxBuffer = stringstream(string(reinterpret_cast<char*>((char*)mesh.faces), fNum * sizeof(Face)));
			tdxBuffer = stringstream(string(reinterpret_cast<char*>((char*)mesh.tvFace), fNum * sizeof(TVFace)));
			ndxBuffer = stringstream(string(reinterpret_cast<char*>((char*)mesh_ns->GetFaceArray()), fNum * sizeof(MeshNormalFace)));
			mtaBuffer = stringstream(string(reinterpret_cast<char*>((char*)&meshMeta), sizeof(MaxMeshMetaData)));
		}

		// Packaging
		sprintf_s(outputNameBuffer, sizeof outputNameBuffer, "%s\\%S.mxo", cachePath.c_str(), node->GetName());
		if (checkpoint) sprintf_s(outputNameBuffer, sizeof outputNameBuffer, "%s\\%S-%s.mxo", cachePath.c_str(), node->GetName(), gtfrmtt());

		// Remove Package If Exists
		std::remove(outputNameBuffer);

		// Compressing
		Zipper zipper(_strlwr(outputNameBuffer));
		if (cacheBufferingMode == DISK_CACHE_BUFFERING_MODE)
		{
			zipper.add(tempAddr + "\\max-mesh.vtx", compressionMode);
			zipper.add(tempAddr + "\\max-mesh.nrm", compressionMode);
			zipper.add(tempAddr + "\\max-mesh.tex", compressionMode);
			zipper.add(tempAddr + "\\max-mesh.idx", compressionMode);
			zipper.add(tempAddr + "\\max-mesh.tdx", compressionMode);
			zipper.add(tempAddr + "\\max-mesh.ndx", compressionMode);
			zipper.add(tempAddr + "\\max-mesh.mta", compressionMode);
		}
		if (cacheBufferingMode == MEMORY_CACHE_BUFFERING_MODE)
		{
			zipper.add(vtxBuffer, "max-mesh.vtx", compressionMode);
			zipper.add(nrmBuffer, "max-mesh.nrm", compressionMode);
			zipper.add(texBuffer, "max-mesh.tex", compressionMode);
			zipper.add(idxBuffer, "max-mesh.idx", compressionMode);
			zipper.add(tdxBuffer, "max-mesh.tdx", compressionMode);
			zipper.add(ndxBuffer, "max-mesh.ndx", compressionMode);
			zipper.add(mtaBuffer, "max-mesh.mta", compressionMode);
		}
		zipper.close();

		// Clear Up
		if (cacheBufferingMode == DISK_CACHE_BUFFERING_MODE) {
			filesystem::remove(tempAddr + "\\max-mesh.vtx");
			filesystem::remove(tempAddr + "\\max-mesh.nrm");
			filesystem::remove(tempAddr + "\\max-mesh.tex");
			filesystem::remove(tempAddr + "\\max-mesh.idx");
			filesystem::remove(tempAddr + "\\max-mesh.tdx");
			filesystem::remove(tempAddr + "\\max-mesh.ndx");
			filesystem::remove(tempAddr + "\\max-mesh.mta");
		}
		if (cacheBufferingMode == MEMORY_CACHE_BUFFERING_MODE) {
			vtxBuffer.clear(); stringstream().swap(vtxBuffer);
			nrmBuffer.clear(); stringstream().swap(nrmBuffer);
			texBuffer.clear(); stringstream().swap(texBuffer);
			idxBuffer.clear(); stringstream().swap(idxBuffer);
			tdxBuffer.clear(); stringstream().swap(tdxBuffer);
			ndxBuffer.clear(); stringstream().swap(ndxBuffer);
			mtaBuffer.clear(); stringstream().swap(mtaBuffer);
		}

		DebugLog(L"Object [%s] successfully cached to %S in %f ms", node->GetName(), outputNameBuffer, profiler.ElapsedMilliseconds());
		return true;
	}

	DebugLog(L"Caching object [%s] failed.", node->GetName());
	return false;
}
bool GenerateNodeFromCache(const wchar_t* mxm_package)
{
	profiler.Reset(); profiler.Start();
	DebugLog(L"Restoring object from cache file [%s]...", mxm_package);

	if (restoreMode == RESTORE_CACHE_MODE_SINGLE_THREAD)
		DebugLog(L"Config `Restore Mode` = RESTORE_CACHE_MODE_SINGLE_THREAD");
	if (restoreMode == RESTORE_CACHE_MODE_MULTI_THREAD)
		DebugLog(L"Config `Restore Mode` = RESTORE_CACHE_MODE_MULTI_THREAD");

	// Convert Package Name
	wstring mxm_package_ws(mxm_package);
	string mxm_package_str(mxm_package_ws.begin(), mxm_package_ws.end());

	// Create Object
	TimeValue t = GetCOREInterface()->GetTime();
	INode* newNode = CreateObjectInScene(GEOMOBJECT_CLASS_ID, EPOLYOBJ_CLASS_ID);
	PolyObject* obj = (PolyObject*)newNode->GetObjectRef();
	MNMesh& mesh = obj->GetMesh();

	// Import Mesh
	Unzipper unzipper(mxm_package_str.c_str());
	BUFFER mta_buffer, vtx_buffer, nrm_buffer, tex_buffer, idx_buffer, tdx_buffer, ndx_buffer;

	// Extracting
	unzipper.extractEntryToMemory("max-mesh.mta", mta_buffer);
	unzipper.extractEntryToMemory("max-mesh.vtx", vtx_buffer);
	unzipper.extractEntryToMemory("max-mesh.nrm", nrm_buffer);
	unzipper.extractEntryToMemory("max-mesh.tex", tex_buffer);
	unzipper.extractEntryToMemory("max-mesh.idx", idx_buffer);
	unzipper.extractEntryToMemory("max-mesh.tdx", tdx_buffer);
	unzipper.extractEntryToMemory("max-mesh.ndx", ndx_buffer);
	unzipper.close();

	// Creating Mesh
	Mesh* newMesh = new Mesh();
	newMesh->SpecifyNormals(); // Needed?
	MeshNormalSpec* mesh_ns = newMesh->GetSpecifiedNormals();
	MaxMeshMetaData* meshMeta = (MaxMeshMetaData*)mta_buffer.data();

	// Set Configs
	newNode->SetName(StringGetWideChar(meshMeta->name));
	newNode->SetNodeTM(t, meshMeta->tm);
	newNode->SetWireColor(meshMeta->col);

	// Allocate Sizes
	newMesh->setNumVerts(meshMeta->vNum);
	mesh_ns->SetNumNormals(meshMeta->nNum);
	newMesh->setNumTVerts(meshMeta->tNum);
	newMesh->setNumFaces(meshMeta->fNum);
	newMesh->setNumTVFaces(meshMeta->fNum);
	mesh_ns->SetNumFaces(meshMeta->fNum);

	// Copy Buffers
	if (restoreMode == RESTORE_CACHE_MODE_SINGLE_THREAD)
	{
		memcpy(newMesh->verts, vtx_buffer.data(), vtx_buffer.size());
		memcpy(mesh_ns->GetNormalArray(), nrm_buffer.data(), nrm_buffer.size());
		memcpy(newMesh->tVerts, tex_buffer.data(), tex_buffer.size());
		memcpy(newMesh->faces, idx_buffer.data(), idx_buffer.size());
		memcpy(newMesh->tvFace, tdx_buffer.data(), tdx_buffer.size());
		memcpy(mesh_ns->GetFaceArray(), ndx_buffer.data(), ndx_buffer.size());
	}
	if (restoreMode == RESTORE_CACHE_MODE_MULTI_THREAD)
	{
		Point3* vertBuffer						= (Point3*)vtx_buffer.data();
		Point3* normalBuffer					= (Point3*)nrm_buffer.data();
		UVVert* textureBuffer					= (UVVert*)tex_buffer.data();
		Face* faceBuffer						= (Face*)idx_buffer.data();
		TVFace* textureFaceBuffer				= (TVFace*)tdx_buffer.data();
		MeshNormalFace* normalFaceBuffer		= (MeshNormalFace*)ndx_buffer.data();

		MULTI_THREAD_LOOP_BEGIN(meshMeta->vNum)
		newMesh->verts[i] = vertBuffer[i];
		MULTI_THREAD_LOOP_END

		MULTI_THREAD_LOOP_BEGIN(meshMeta->nNum)
		mesh_ns->GetNormalArray()[i] = normalBuffer[i];
		MULTI_THREAD_LOOP_END

		MULTI_THREAD_LOOP_BEGIN(meshMeta->tNum)
		newMesh->tVerts[i] = textureBuffer[i];
		MULTI_THREAD_LOOP_END

		MULTI_THREAD_LOOP_BEGIN(meshMeta->fNum)
		newMesh->faces[i] = faceBuffer[i];
		newMesh->tvFace[i] = textureFaceBuffer[i];
		mesh_ns->GetFaceArray()[i] = normalFaceBuffer[i];
		MULTI_THREAD_LOOP_END
	}
	
	// Finalaizing
	mesh.SetFromTri(*newMesh);
	mesh.InvalidateGeomCache();
	mesh.InvalidateTopoCache();

	// Merge Tris
	mesh.MakePolyMesh();
	
	// Releasing
	newMesh->FreeAll();
	delete newMesh; BUFFER_FREE(mta_buffer);
	BUFFER_FREE(vtx_buffer); BUFFER_FREE(nrm_buffer);
	BUFFER_FREE(tex_buffer); BUFFER_FREE(idx_buffer);
	BUFFER_FREE(tdx_buffer); BUFFER_FREE(ndx_buffer);

	// Update 
	GetCOREInterface()->RedrawViews(t);
	obj->NotifyDependents(FOREVER, ALL_CHANNELS, REFMSG_CHANGE);

	DebugLog(L"Cache [%s] successfully restored to %s in %f ms",
		mxm_package,
		newNode->GetName(),
		profiler.ElapsedMilliseconds());

	return true;
}
bool GenerateNewPolyFromCache(const wchar_t* mxm_package, INode* node)
{
	theHold.Begin();
	profiler.Reset(); profiler.Start();
	DebugLog(L"Restoring mesh from cache file [%s]...", mxm_package);

	if (restoreMode == RESTORE_CACHE_MODE_SINGLE_THREAD)
		DebugLog(L"Config `Restore Mode` = RESTORE_CACHE_MODE_SINGLE_THREAD");
	if (restoreMode == RESTORE_CACHE_MODE_MULTI_THREAD)
		DebugLog(L"Config `Restore Mode` = RESTORE_CACHE_MODE_MULTI_THREAD");

	// Convert Package Name
	wstring mxm_package_ws(mxm_package);
	string mxm_package_str(mxm_package_ws.begin(), mxm_package_ws.end());

	// Get Poly Object
	PolyObject* obj = (PolyObject*)node->GetObjectRef();

	// Geometry Validation
	if (obj->FindBaseObject()->SuperClassID() != GEOMOBJECT_CLASS_ID) return false;

	// Editable Poly Validation 
	if (obj->FindBaseObject()->ClassID() != EPOLYOBJ_CLASS_ID) return false;

	// Create Undo/Redo Backup
	theHold.Put(new RestoreMeshOp(obj, mxm_package, node));

	// Get Mesh
	MNMesh& mesh = obj->GetMesh();

	// Import Mesh
	Unzipper unzipper(mxm_package_str.c_str());
	BUFFER mta_buffer, vtx_buffer, nrm_buffer, tex_buffer, idx_buffer, tdx_buffer, ndx_buffer;

	// Extracting
	unzipper.extractEntryToMemory("max-mesh.mta", mta_buffer);
	unzipper.extractEntryToMemory("max-mesh.vtx", vtx_buffer);
	unzipper.extractEntryToMemory("max-mesh.nrm", nrm_buffer);
	unzipper.extractEntryToMemory("max-mesh.tex", tex_buffer);
	unzipper.extractEntryToMemory("max-mesh.idx", idx_buffer);
	unzipper.extractEntryToMemory("max-mesh.tdx", tdx_buffer);
	unzipper.extractEntryToMemory("max-mesh.ndx", ndx_buffer);
	unzipper.close();

	// Creating Mesh
	Mesh* newMesh = new Mesh();
	newMesh->SpecifyNormals();
	MeshNormalSpec* mesh_ns = newMesh->GetSpecifiedNormals();
	MaxMeshMetaData* meshMeta = (MaxMeshMetaData*)mta_buffer.data();

	// Allocate Sizes
	newMesh->setNumVerts(meshMeta->vNum);
	mesh_ns->SetNumNormals(meshMeta->nNum);
	newMesh->setNumTVerts(meshMeta->tNum);
	newMesh->setNumFaces(meshMeta->fNum);
	newMesh->setNumTVFaces(meshMeta->fNum);
	mesh_ns->SetNumFaces(meshMeta->fNum);

	// Copy Buffers
	if (restoreMode == RESTORE_CACHE_MODE_SINGLE_THREAD)
	{
		memcpy(newMesh->verts, vtx_buffer.data(), vtx_buffer.size());
		memcpy(mesh_ns->GetNormalArray(), nrm_buffer.data(), nrm_buffer.size());
		memcpy(newMesh->tVerts, tex_buffer.data(), tex_buffer.size());
		memcpy(newMesh->faces, idx_buffer.data(), idx_buffer.size());
		memcpy(newMesh->tvFace, tdx_buffer.data(), tdx_buffer.size());
		memcpy(mesh_ns->GetFaceArray(), ndx_buffer.data(), ndx_buffer.size());
	}
	if (restoreMode == RESTORE_CACHE_MODE_MULTI_THREAD)
	{
		Point3* vertBuffer = (Point3*)vtx_buffer.data();
		Point3* normalBuffer = (Point3*)nrm_buffer.data();
		UVVert* textureBuffer = (UVVert*)tex_buffer.data();
		Face* faceBuffer = (Face*)idx_buffer.data();
		TVFace* textureFaceBuffer = (TVFace*)tdx_buffer.data();
		MeshNormalFace* normalFaceBuffer = (MeshNormalFace*)ndx_buffer.data();

		MULTI_THREAD_LOOP_BEGIN(meshMeta->vNum)
		newMesh->verts[i] = vertBuffer[i];
		MULTI_THREAD_LOOP_END
		MULTI_THREAD_LOOP_BEGIN(meshMeta->nNum)
		mesh_ns->GetNormalArray()[i] = normalBuffer[i];
		MULTI_THREAD_LOOP_END
		MULTI_THREAD_LOOP_BEGIN(meshMeta->tNum)
		newMesh->tVerts[i] = textureBuffer[i];
		MULTI_THREAD_LOOP_END
		MULTI_THREAD_LOOP_BEGIN(meshMeta->fNum)
		newMesh->faces[i] = faceBuffer[i];
		newMesh->tvFace[i] = textureFaceBuffer[i];
		mesh_ns->GetFaceArray()[i] = normalFaceBuffer[i];
		MULTI_THREAD_LOOP_END
	}

	// Finalaizing
	mesh.SetFromTri(*newMesh);
	mesh.InvalidateGeomCache();
	mesh.InvalidateTopoCache();

	// Merge Tris
	mesh.MakePolyMesh();

	// Releasing
	newMesh->FreeAll();
	delete newMesh; BUFFER_FREE(mta_buffer);
	BUFFER_FREE(vtx_buffer); BUFFER_FREE(nrm_buffer);
	BUFFER_FREE(tex_buffer); BUFFER_FREE(idx_buffer);
	BUFFER_FREE(tdx_buffer); BUFFER_FREE(ndx_buffer);

	// Update 
	GetCOREInterface()->RedrawViews(GetCOREInterface()->GetTime());
	obj->NotifyDependents(FOREVER, ALL_CHANNELS, REFMSG_CHANGE);

	DebugLog(L"Cache [%s] successfully restored to %s in %f ms", mxm_package, node->GetName(), profiler.ElapsedMilliseconds());

	theHold.Accept(L"MXMesh :: RestoreMesh");
	return true;
}

// Maxscript Exposed API
MaxMeshMXS(Cache, "Cache");
Value* Cache_api(Value** arg_list, int count)
{
	if (count == 1)
	{
		INode* node = arg_list[0]->to_node();
		if (CacheMeshToDisk(node)) return &true_value;
		else return &false_value;
	}
	else
	{
		throw RuntimeError(L"Invalid Inputs, Correct : MXMesh.Cache <node>"); return &false_value;
	}
}
MaxMeshMXS(Checkpoint, "Checkpoint");
Value* Checkpoint_api(Value** arg_list, int count)
{
	if (count == 1)
	{
		INode* node = arg_list[0]->to_node();
		if (CacheMeshToDisk(node, true)) return &true_value;
		else return &false_value;
	}
	else
	{
		throw RuntimeError(L"Invalid Inputs, Correct : MXMesh.Cache <node>"); return &false_value;
	}
}
MaxMeshMXS(SetCachePath, "SetCachePath");
Value* SetCachePath_api(Value** arg_list, int count)
{
	if (count == 1)
	{
		const wchar_t* cachePathVal = arg_list[0]->to_string();
		wstring cachePathValws(cachePathVal);
		cachePath = string(cachePathValws.begin(), cachePathValws.end());

		DebugLog(L"MXMesh cache path has been set to `%s`\n", cachePathVal);
		return &ok;
	}
	else
	{
		throw RuntimeError(L"Invalid Inputs, Correct : MXMesh.SetCachePath <path>"); return &false_value;
	}
}
MaxMeshMXS(GetCachePath, "GetCachePath");
Value* GetCachePath_api(Value** arg_list, int count)
{
	if (count == 0)
	{
		String* _str = new String(StringGetWideChar(cachePath.c_str()));
		return _str;
	}
	else
	{
		throw RuntimeError(L"Invalid Inputs, Correct : <string> MXMesh.GetCachePath()"); return &false_value;
	}
}
MaxMeshMXS(Restore, "Restore");
Value* Restore_api(Value** arg_list, int count)
{
	if (count == 1)
	{
		const wchar_t* cacheFile = arg_list[0]->to_string();
		if (GenerateNodeFromCache(cacheFile)) return &true_value;
		else return &false_value;
	}
	else
	{
		throw RuntimeError(L"Invalid Inputs, Correct : MXMesh.Restore <cache_file>"); return &false_value;
	}
}
MaxMeshMXS(RestoreMesh, "RestoreMesh");
Value* RestoreMesh_api(Value** arg_list, int count)
{
	if (count == 2)
	{
		const wchar_t* cacheFile = arg_list[0]->to_string();
		INode* caheNode = arg_list[1]->to_node();
		if (GenerateNewPolyFromCache(cacheFile, caheNode)) return &true_value;
		else return &false_value;
	}
	else
	{
		throw RuntimeError(L"Invalid Inputs, Correct : MXMesh.RestoreMesh <cache_file> <node>"); return &false_value;
	}
}
MaxMeshMXS(Purge, "Purge");
Value* Purge_api(Value** arg_list, int count)
{
	if (count == 0)
	{
		ExecuteMAXScriptScript(L"gc()", MAXScript::ScriptSource::NonEmbedded, TRUE);
		if (filesystem::exists(cachePath) && filesystem::is_directory(cachePath))
			for (auto const& entry : filesystem::recursive_directory_iterator(cachePath)) 
			{
				string ext = entry.path().extension().string(); String2Lower(ext);
				if (filesystem::is_regular_file(entry) && ext == ".mxo") filesystem::remove(entry);
			}
		return &ok;
	}
	else
	{
		throw RuntimeError(L"Invalid Inputs, Correct : MXMesh.Purge()"); return &false_value;
	}
}
MaxMeshMXS(SetRestoreMode, "SetRestoreMode");
Value* SetRestoreMode_api(Value** arg_list, int count)
{
	if (count == 1)
	{
		auto option = arg_list[0]->to_string();
		if (wcscmp(option, L"single") == 0) {
			restoreMode = RESTORE_CACHE_MODE_SINGLE_THREAD;
			DebugLog(L"MXMesh : Restoring Mode has been set to single thread mode.");
			return &ok;
		}
		if (wcscmp(option, L"multi") == 0) {
			restoreMode = RESTORE_CACHE_MODE_MULTI_THREAD;
			DebugLog(L"MXMesh : Restoring Mode has been set to multi thread mode.");
			return &ok;
		}
		return &false_value;
	}
	else
	{
		throw RuntimeError(L"Invalid Inputs, Correct : MXMesh.SetRestoreMode [#single][#multi]"); return &false_value;
	}
}
MaxMeshMXS(SetCacheBufferingMode, "SetCacheBufferingMode");
Value* SetCacheBufferingMode_api(Value** arg_list, int count)
{
	if (count == 1)
	{
		auto option = arg_list[0]->to_string();
		if (wcscmp(option, L"disk") == 0) {
			cacheBufferingMode = DISK_CACHE_BUFFERING_MODE;
			DebugLog(L"MXMesh : Cache Buffering Mode has been set to disk mode.");
			return &ok;
		}
		if (wcscmp(option, L"memory") == 0) {
			cacheBufferingMode = MEMORY_CACHE_BUFFERING_MODE;
			DebugLog(L"MXMesh : Cache Buffering Mode has been set to memory mode.");
			return &ok;
		}
		return &false_value;
	}
	else
	{
		throw RuntimeError(L"Invalid Inputs, Correct : MXMesh.SetCacheBufferingMode [#disk][#memory]"); return &false_value;
	}
}
MaxMeshMXS(SetCompressionMode, "SetCompressionMode");
Value* SetCompressionMode_api(Value** arg_list, int count)
{
	if (count == 1)
	{
		auto option = arg_list[0]->to_string();
		if (wcscmp(option, L"faster") == 0) {
			compressionMode = Zipper::Faster;
			DebugLog(L"MXMesh : Compression Mode has been set to faster.");
			return &ok;
		}
		if (wcscmp(option, L"better") == 0) {
			compressionMode = Zipper::Better;
			DebugLog(L"MXMesh : Compression Mode has been set to better.");
			return &ok;
		}
		return &false_value;
	}
	else
	{
		throw RuntimeError(L"Invalid Inputs, Correct : MXMesh.SetCompressionMode [#faster][#better]"); return &false_value;
	}
}
MaxMeshMXS(CopyMesh, "CacheToMemory");
Value* CopyMesh_api(Value** arg_list, int count)
{
	throw RuntimeError(L"<< Not Implemented Yet >>"); return &false_value;
}
MaxMeshMXS(PasteMesh, "RestoreFromMemory");
Value* PasteMesh_api(Value** arg_list, int count)
{
	throw RuntimeError(L"<< Not Implemented Yet >>"); return &false_value;
}
MaxMeshMXS(SetDebugMode, "SetDebugMode");
Value* SetDebugMode_api(Value** arg_list, int count)
{
	if (count == 1)
	{
		DebugMode = arg_list[0]->to_bool();
		return &ok;
	}
	else
	{
		throw RuntimeError(L"Invalid Inputs, Correct : MXMesh.SetDebugMode <bool>"); return &false_value;
	}
}

// MaxSDK Functions
extern "C" __declspec(dllexport) const TCHAR* LibDescription()
{
	return PLUGIN_DESCRIPTION;
}
extern "C" __declspec(dllexport) int LibNumberClasses()
{
	return 0;
}
extern "C" __declspec(dllexport) ClassDesc* LibClassDesc(int i)
{
	return 0;
}
extern "C" __declspec(dllexport) ULONG LibVersion()
{
	return Get3DSMAXVersion();
}
extern "C" __declspec(dllexport) int LibInitialize(void)
{
	maxInterface = GetCOREInterface();
	cachePath = filesystem::temp_directory_path().string();
	return TRUE;
}
extern "C" __declspec(dllexport) int LibShutdown(void)
{
	return TRUE;
}
extern "C" __declspec(dllexport) ULONG CanAutoDefer()
{
	return FALSE;
}