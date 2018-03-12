#include "adaptation.h"
#include <fstream>

char CLogger::s_szHeaderSim[] = "time_exp, time_real_elapse"
							", position.x, position.y, position.z"
							", tangent.i, tangent.j, tangent.k"
							", lateral.i, lateral.j, lateral.k"
							", vel"
							", visualState, audioState"
							", acc"
							", sus1, sus2, sus3, sus4"
							", velbrake"
							", latAccel"
							", fidelity"
							", angularv.x, angularv.y, angularv.z";

char CLogger::s_szHeaderVrLink[] = "time_exp, time_real_elapse"
							", loc.x, loc.y, loc.z"
							", vel.x, vel.y, vel.z"
							", acc.x, acc.y, acc.z"
							", ori.psi, ori.theta, ori.phi"
							", rot.x, rot.y, rot.z";

CLogger g_SimLogger("sim.log", CLogger::s_szHeaderSim);
CLogger g_VrlinkLogger("vrlink.log", CLogger::s_szHeaderVrLink);
CLogger g_VrlinkLoggerRaw("vrlinkraw.log", CLogger::s_szHeaderVrLink);
CLogger g_VrlinkLoggerLast("vrlinklast.log", CLogger::s_szHeaderVrLink);

CLogger::CLogger(const char* fileName, const char* szHeader) : m_fileName(fileName)
{
	Logoutf(szHeader);
}

CLogger::~CLogger()
{
	std::fstream o(m_fileName.c_str(), std::ios::out|std::ios::trunc);
	for (std::list<const char*>::iterator it = m_lstLogs.begin(); it != m_lstLogs.end(); it ++)
	{
		const char* buffer = *it;
		o << buffer;
		free((void *)buffer);
	}
}

void CLogger::Logoutf(const char* format, ...)
{
	va_list arglist;
	va_start(arglist, format);
	int cap = 256;
	char* buffer = (char *)malloc(sizeof(char) * cap);
	bool gen = false;
	do
	{
		int l = _vsnprintf(buffer, cap, format, arglist);
		gen = (l > 0 && l < cap);
		if (!gen)
		{
			cap += cap;
			buffer = (char *)realloc(buffer, sizeof(char) * cap);
		}
	} while(!gen);

	//OutputDebugString(buffer);
	//free(buffer);
	m_lstLogs.push_back(buffer);
}

void CLogger::Logout(DtTime t, DWORD rtElapse, const struct ExternalDriverState& s)
{
	//bbox is missing
	Logoutf("\n%.3f, %d"
			",%E,%E,%E"
			",%E,%E,%E"
			",%E,%E,%E"
			",%E"
			",%E,%E"
			",%E"
			",%E,%E,%E,%E"
			",%E"
			",%E"
			",%d"
			",%E,%E,%E"
			, t, rtElapse
			, s.position.x, s.position.y, s.position.z
			, s.tangent.i, s.tangent.j, s.tangent.k
			, s.lateral.i, s.lateral.j, s.lateral.k
			, s.vel
			, s.visualState, s.audioState
			, s.acc
			, s.suspStif, s.suspDamp, s.tireStif, s.tireDamp
			, s.velBrake
			, s.latAccel
			, s.dynaFidelity
			, s.angularVel.i, s.angularVel.j, s.angularVel.k);
}
void CLogger::Logout(DtTime t, DWORD rtElapse, const struct ExternalDriverStateTran& s)
{
	Logoutf("\n%.3f, %d"
			",%E,%E,%E"
			",%E,%E,%E"
			",%E,%E,%E"
			",%E,%E,%E"
			",%E,%E,%E"
			, t, rtElapse
			, s.loc.x(), s.loc.y(), s.loc.z()
			, s.vel.x(), s.vel.y(), s.vel.z()
			, s.acc.x(), s.acc.y(), s.acc.z()
			, s.ori.psi(), s.ori.theta(), s.ori.phi()
			, s.rot.x(), s.rot.y(), s.rot.z());
}