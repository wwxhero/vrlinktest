//adpatation layer between CVED and VRLink
//typedef DtVector TPoint3D;
//typedef DtVector32 TVector3D;

#include <vl/exerciseConn.h>
#include <vl/exerciseConnInitializer.h>

#include <vlutil/vlProcessControl.h>
#include <vlutil/vlMiniDumper.h>

#include <vl/reflectedEntityList.h>
#include <vl/entityStateRepository.h>
#include <vl/reflectedEntity.h>
#include <vl/fireInteraction.h>
#include <vl/topoView.h>
#include <vl/fomMapper.h>
#include <iostream>
#include <vl/environmentProcessRepository.h>
#include <vl/reflectedEnvironmentProcess.h>
#include <vl/reflectedEnvironmentProcessList.h>
#include <matrix/vlQuaternion.h>
#include <assert.h>
#include <list>

//glm utilities
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#include <glm/gtx/transform.hpp>
#include <glm/gtx/transform2.hpp>
#include <glm/gtx/euler_angles.hpp>

typedef short TU16b;
typedef char cvTerQueryHint;
typedef int EDynaFidelity;
#define PI 3.1416
#ifdef _DEBUG
#define _TEST_FRAME_TB
#endif



typedef struct TPoint3D_tag { /* {secret} */
	double x;
	double y;
	double z;
} TPoint3D;

typedef struct TVector3D_tag {   /* {secret} */
	double i;
	double j;
	double k;
} TVector3D;

#ifdef SIM
static const TVector3D c_tangent0 = {0, -1, 0};
static const TVector3D c_lateral0 = {-1, 0, 0};
#else
static const TVector3D c_tangent0 = {0, 0, 1};
static const TVector3D c_lateral0 = {1, 0, 0};
#endif

static const DtDeadReckonTypes c_drkDefault = DtDrDrmRpw;
struct ExternalDriverState {
	TPoint3D          position;     /* object's position */
	TVector3D         tangent;      /* tangent vector (normalized) */
	TVector3D         lateral;      /* lateral vector (normalized) */
	TPoint3D          boundBox[2];   /* object's bounding box */
	double            vel;          /* velocity along tangent */
	TU16b             visualState;
	TU16b             audioState;

	/* the following are needed for fake driver vehicle dynamics */
	double            acc;          /* acceleration along tangent */
	double            suspStif;     /* suspension stiffness */
	double            suspDamp;     /* suspension dampness */
	double            tireStif;     /* tire stiffness */
	double            tireDamp;     /* tire dampness */
	double            velBrake;
	cvTerQueryHint    posHint[4];
	double            latAccel;     /* vehicle's current lat accel  */
	EDynaFidelity     dynaFidelity; /* vehicle's dynamics fidelity  */
	//External Driver State Matches with DynObjState above this point
	//....new elements should be added after here.

	TVector3D         angularVel;   /* object's angular velocity */

};


inline void mockDyna(DtTime t, struct ExternalDriverState& s)
{
	const float scale = 5.0f;
	const float omega = 0.1f;
	s.position.x = scale * cos(t * omega);
	s.position.y = 0.0f;
	s.position.z = scale * sin(t * omega);

	s.tangent.i = -sin(t * omega);
	s.tangent.j = 0.0f;
	s.tangent.k = cos(t * omega);

	s.lateral.i = cos(t * omega);
	s.lateral.j = 0.0f;
	s.lateral.k = sin(t * omega);

	memset(s.boundBox, 0, sizeof(TPoint3D)*2); //fix me: no idea about the bbox

	s.vel = scale * omega;
	s.visualState = 0; //fix me: no idea
	s.audioState = 0; //fix me: no idea

	s.acc = 0;
	s.suspStif = 0; //fix me
	s.suspDamp = 0;
	s.tireStif = 0;
	s.tireDamp = 0;
	s.velBrake = 0;
	memset(s.posHint, 0, 4 * sizeof(cvTerQueryHint));
	s.dynaFidelity = 0;

	//suppose in euler angle
	s.angularVel.i = 0;
	s.angularVel.j = 0;
	s.angularVel.k = (2 * PI) / omega;

}

inline void mockDyna_static(DtTime t, struct ExternalDriverState& s)
{
	const float scale = 5.0f;
	const float omega = 0.01f;
	s.position.x = 0.0f;
	s.position.y = 0.0f;
	s.position.z = 0.0f;

	s.tangent.i = 0.0f;
	s.tangent.j = 0.0f;
	s.tangent.k = 1.0f;

	s.lateral.i = -1.0f;
	s.lateral.j =  0.0f;
	s.lateral.k =  0.0f;

	memset(s.boundBox, 0, sizeof(TPoint3D)*2); //fix me: no idea about the bbox

	s.vel = scale * omega;
	s.visualState = 0; //fix me: no idea
	s.audioState = 0; //fix me: no idea

	s.acc = 0;
	s.suspStif = 0; //fix me
	s.suspDamp = 0;
	s.tireStif = 0;
	s.tireDamp = 0;
	s.velBrake = 0;
	memset(s.posHint, 0, 4 * sizeof(cvTerQueryHint));
	s.dynaFidelity = 0;

	//suppose in euler angle
	s.angularVel.i = 0;
	s.angularVel.j = 0;
	s.angularVel.k = 0;
}

struct ExternalDriverStateTran {
	DtVector32  vel;
	DtVector32  acc;
	DtTaitBryan ori;
	DtVector32  rot;
	DtVector    loc;
};

inline void Frame2TaitBryan(const TVector3D& tangent, const TVector3D& lateral, const TVector3D& tangent_prime, const TVector3D& lateral_prime, DtTaitBryan& ori)
{
	//todo: generate tait-bryan euler angle from [t,l,b] to [t', l', b']
	glm::dvec3 t(tangent.i, tangent.j, tangent.k);
	glm::dvec3 l(lateral.i, lateral.j, lateral.k);
	glm::dvec3 b = glm::cross(t, l);
	glm::dmat3 f;
	f[0] = t;
	f[1] = l;
	f[2] = b;

	glm::dvec3 t_prime(tangent_prime.i, tangent_prime.j, tangent_prime.k);
	glm::dvec3 l_prime(lateral_prime.i, lateral_prime.j, lateral_prime.k);
	glm::dvec3 b_prime = glm::cross(t_prime, l_prime);
	glm::dmat3 f_prime;
	f_prime[0] = t_prime;
	f_prime[1] = l_prime;
	f_prime[2] = b_prime;

	glm::dmat3 r = f_prime * glm::inverse(f);

	glm::dquat q = glm::quat_cast(r);
	DtQuaternion q_vrlink(q.w, q.x, q.y, q.z);
	ori = q_vrlink; //this call might have performance overhead
}

inline void TaitBryan2Frame(const DtTaitBryan& ori, const TVector3D& tangent, const TVector3D& lateral, TVector3D& tangent_prime, TVector3D& lateral_prime)
{
	//todo: generate tangent and lateral [t', l', b'] from orientation, and [t, l, b]
	double psi = ori.psi();
	double theta = ori.theta();
	double phi = ori.phi();
	double c1 = cos(psi);
	double s1 = sin(psi);
	double c2 = cos(theta);
	double s2 = sin(theta);
	double c3 = cos(phi);
	double s3 = sin(phi);
	glm::dmat3 r(c1*c2					,c2*s1					,-s2
				,c1*s2*s3 - c3*s1		,c1*c3 + s1*s2*s3		,c2*s3
				,s1*s3 + c1*c3*s2		,c3*s1*s2 - c1*s3		,c2*c3);
	glm::dvec3 t(tangent.i, tangent.j, tangent.k);
	glm::dvec3 t_prime = r * t;
	tangent_prime.i = t_prime.x;
	tangent_prime.j = t_prime.y;
	tangent_prime.k = t_prime.z;

	glm::dvec3 l(lateral.i, lateral.j, lateral.k);
	glm::dvec3 l_prime = r * l;
	lateral_prime.i = l_prime.x;
	lateral_prime.j = l_prime.y;
	lateral_prime.k = l_prime.z;

}
//bbox is not transformed
inline void Transform(const struct ExternalDriverState& src, struct ExternalDriverStateTran& dst)
{
	dst.loc.setX(src.position.x);
	dst.loc.setY(src.position.y);
	dst.loc.setZ(src.position.z);

	dst.vel.setX(src.vel * src.tangent.i);
	dst.vel.setY(src.vel * src.tangent.j);
	dst.vel.setZ(src.vel * src.tangent.k);

	dst.acc.setX(src.acc * src.tangent.i + src.latAccel * src.lateral.i);
	dst.acc.setY(src.acc * src.tangent.j + src.latAccel * src.lateral.j);
	dst.acc.setZ(src.acc * src.tangent.k + src.latAccel * src.lateral.k);

	Frame2TaitBryan(c_tangent0, c_lateral0, src.tangent, src.lateral, dst.ori);
#ifdef _TEST_FRAME_TB
	const double epsilon = 0.00001;//FLT_EPSILON;
	TVector3D tangent_prime;
	TVector3D lateral_prime;
	TaitBryan2Frame(dst.ori, c_tangent0, c_lateral0, tangent_prime, lateral_prime);
	double cos_t = src.tangent.i * tangent_prime.i
			+ src.tangent.j * tangent_prime.j
			+ src.tangent.k * tangent_prime.k;
	double cos_l = src.lateral.i * lateral_prime.i
			+ src.lateral.j * lateral_prime.j
			+ src.lateral.k * lateral_prime.k;
	assert(cos_t < 1+epsilon
		 && cos_t > 1-epsilon);
	assert(cos_l < 1+epsilon
		&& cos_l > 1-epsilon);
#endif

	dst.rot.setX(src.angularVel.i);
	dst.rot.setY(src.angularVel.j);
	dst.rot.setZ(src.angularVel.k);

}

inline void Transform(const struct ExternalDriverStateTran& src, struct ExternalDriverState& dst)
{
	dst.position.x = src.loc.x();
	dst.position.y = src.loc.y();
	dst.position.z = src.loc.z();

	DtVector32 v = src.vel;
	dst.vel = sqrt(v.magnitudeSquared());
	v.normalize();
	dst.tangent.i = v.x();
	dst.tangent.j = v.y();
	dst.tangent.k = v.z();

	TaitBryan2Frame(src.ori, c_tangent0, c_lateral0, dst.tangent, dst.lateral);

	DtVector32 a = src.acc;
	DtVector32 t = v;
	DtVector32 l(dst.lateral.i, dst.lateral.j, dst.lateral.k);

	dst.acc = a.dotProduct(t);
	dst.latAccel = a.dotProduct(l);

	dst.angularVel.i = src.rot.x();
	dst.angularVel.j = src.rot.y();
	dst.angularVel.k = src.rot.z();
}

class CLogger
{
public:
	CLogger(const char* fileName, const char* szHeader);
	~CLogger();
	void Logout(DtTime t, DWORD rtElapse, const struct ExternalDriverState& s);
	void Logout(DtTime t, DWORD rtElapse, const struct ExternalDriverStateTran& s);
	static char s_szHeaderSim[];
	static char s_szHeaderVrLink[];
private:
	void Logoutf(const char* format, ...);
	std::list<const char*> m_lstLogs;
	std::string m_fileName;
};

extern CLogger g_SimLogger;
extern CLogger g_VrlinkLogger;

inline void Logout(DtTime t, DWORD rtElapse, const struct ExternalDriverState& s)
{
	g_SimLogger.Logout(t, rtElapse, s);
}

inline void Logout(DtTime t, DWORD rtElapse, const struct ExternalDriverStateTran& s)
{
	g_VrlinkLogger.Logout(t, rtElapse, s);
}