

#define _WIN32_WINNT  0x0500

#include ".\threadlock.h"


/***********************************************************
* WIN32 specific methods
***********************************************************/
#ifdef WIN32

/********************************************************************
* FUNCTION NAME :  Constructor.
*
* DESCRIPTION:     Initializes a critical section object.
*
* INPUTS:	       None.
*
* RETURN VALUE:	   None.
*
********************************************************************/
ThreadLock::ThreadLock(void)
{
	InitializeCriticalSection(&m_lock);
	m_locked = 0;
}


/********************************************************************
* FUNCTION NAME :  Destructor.
*
* DESCRIPTION:     Unlock the mutex and releases all resources
*                  used by an unowned critical section object.
*
* INPUTS:	       None.
*
* RETURN VALUE:	   None.
*
********************************************************************/
ThreadLock::~ThreadLock(void)
{
	if (m_locked)
	{
		Unlock();
	}
	DeleteCriticalSection(&m_lock);
}


/********************************************************************
* FUNCTION NAME :  Lock.
*
* DESCRIPTION:     Waits for ownership of the specified critical
*                  section object. The function returns when the
*                  calling thread is granted ownership.
*
* INPUTS:	       None.
*
* RETURN VALUE:	   None.
*
********************************************************************/
void ThreadLock::Lock()
{
	EnterCriticalSection(&m_lock);
	m_locked = 1;
}


/********************************************************************
* FUNCTION NAME :  Unlock.
*
* DESCRIPTION:     Releases ownership of the specified critical
*				   section object.
*
* INPUTS:	       None.
*
* RETURN VALUE:	   None.
*
********************************************************************/
void ThreadLock::Unlock()
{
	LeaveCriticalSection(&m_lock);
	m_locked = false;
}

#endif /* WIN32 */




/***********************************************************
* GNU/Linux and FreeBSD (x86, PPC) specific methods
***********************************************************/
#if ((defined __linux__) || (defined __FreeBSD__) || (defined __OpenBSD__))


/********************************************************************
* FUNCTION NAME :  Constructor.
*
* DESCRIPTION:     Initializes a critical section object.
*
* INPUTS:	       None.
*
* RETURN VALUE:	   None.
*
********************************************************************/
ThreadLock::ThreadLock()
{
	pthread_mutex_init(&m_mutex, (pthread_mutexattr_t *)0);
	m_locked = 0;
}


/********************************************************************
* FUNCTION NAME :  Destructor.
*
* DESCRIPTION:     Unlock the mutex and releases all resources
*                  used by an unowned critical section object.
*
* INPUTS:	       None.
*
* RETURN VALUE:	   None.
*
********************************************************************/
ThreadLock::~ThreadLock()
{
	if (m_locked)
	{
		unlock();
	}
	pthread_mutex_destroy(&m_mutex);
}


/********************************************************************
* FUNCTION NAME :  Lock.
*
* DESCRIPTION:     Waits for ownership of the specified critical
*                  section object. The function returns when the
*                  calling thread is granted ownership.
*
* INPUTS:	       None.
*
* RETURN VALUE:	   None.
*
********************************************************************/
void ThreadLock::Lock()
{
	pthread_mutex_lock(&m_mutex);
	m_locked = 1;
}


/********************************************************************
* FUNCTION NAME :  Unlock.
*
* DESCRIPTION:     Releases ownership of the specified critical
*				   section object.
*
* INPUTS:	       None.
*
* RETURN VALUE:	   None.
*
********************************************************************/
void ThreadLock::Unlock()
{
	pthread_mutex_unlock(&m_mutex);
	m_locked = 0;
}

#endif /* __linux__ || __FreeBSD__ || __OpenBSD__ */


/********************************************************************
* FUNCTION NAME :  IsLocked.
*
* DESCRIPTION:     Tells if the mutex is locked or not.
*
* INPUTS:	       None.
*
* RETURN VALUE:	   0 - unlocked,   1 - locked.
*
********************************************************************/
int ThreadLock::IsLocked()
{
	return m_locked;
}
