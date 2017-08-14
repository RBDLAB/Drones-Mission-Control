

#pragma once

#include <windows.h>
#include <windef.h>
#include <winbase.h>


/********************************************************************
* CLASS NAME : ThreadLock
*
* DESCRIPTION: A platform independent mutex implementation.
*
********************************************************************/
class ThreadLock
{
public:
	ThreadLock(void);
	~ThreadLock(void);

	/// Function name   : Lock
	/// Description     : Waits for ownership of the specified critical 
	///					  section object.
	/// Return type     : void 
	void Lock();

	/// Function name   : Unlock
	/// Description     : Releases ownership of the specified critical 
	///                   section object.
	/// Return type     : void 
	void Unlock();

	/// Function name   : IsLocked
	/// Description     : returns 1 if this mutex is locked; 0 otherwise.
	/// Return type     : int 
	int IsLocked();

private:
	int m_locked;                // Indicating if the mutex is locked.
	CRITICAL_SECTION m_lock;       // Object provide synchronization similar to 
								   // that provided by mutex objects.
};
