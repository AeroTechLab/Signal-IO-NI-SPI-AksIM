////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2019 Leonardo Consoni <leonardojc@protonmail.com>      //
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include "signal_io.h"

#include <NIDAQmx.h>

#include "threads/threads.h"
#include "threads/khash.h"

#include "debug/data_logging.h"

#define DEBUG_MESSAGE_LENGTH 256

#define INT16_MAX 65536

const size_t TRANSFER_BUFFER_LENGTH = 16;

typedef struct _SignalIOTaskData
{
  TaskHandle clockGenerator;
  TaskHandle misoReader;
  TaskHandle mosiWriter;
  TaskHandle chipSelector;
  Thread threadID;
  volatile bool isRunning;
  uInt16* misoData;
  int32* inputValuesList;
  bool* inputsUsedList;
  uInt16* mosiData;
  int32* outputValuesList;
  bool* outputsUsedList;
  uInt32 channelsNumber;
}
SignalIOTaskData;

typedef SignalIOTaskData* SignalIOTask;  

KHASH_MAP_INIT_INT( TaskInt, SignalIOTask )
static khash_t( TaskInt )* tasksList = NULL;

DECLARE_MODULE_INTERFACE( SIGNAL_IO_INTERFACE ) 

static void* AsyncTransfer( void* );

static SignalIOTask LoadTaskData( char* );
static void UnloadTaskData( SignalIOTask );

static bool CheckTask( SignalIOTask );

int InitDevice( const char* taskConfig )
{
  if( tasksList == NULL ) tasksList = kh_init( TaskInt );
  
  int taskKey = (int) kh_str_hash_func( taskConfig );
  DEBUG_PRINT( "loading task %s (key %d)", taskConfig, taskKey );
  int insertionStatus;
  khint_t newTaskIndex = kh_put( TaskInt, tasksList, taskKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( tasksList, newTaskIndex ) = LoadTaskData( (char*) taskConfig );
    if( kh_value( tasksList, newTaskIndex ) == NULL )
    {
      DEBUG_PRINT( "loading task %s failed", taskConfig );
      EndDevice( taskKey ); 
      return -1;
    }
        
    //DEBUG_PRINT( "new key %d inserted (iterator: %u - total: %u)", kh_key( tasksList, newTaskIndex ), newTaskIndex, kh_size( tasksList ) );
  }
  //else if( insertionStatus == 0 ) { DEBUG_PRINT( "task key %d already exists (iterator %u)", taskKey, newTaskIndex ); }
  
  return (int) kh_key( tasksList, newTaskIndex );
}

void EndDevice( int taskID )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( CheckTask( task ) ) return;
  
  UnloadTaskData( task );
  
  kh_del( TaskInt, tasksList, (khint_t) taskID );
  
  if( kh_size( tasksList ) == 0 )
  {
    kh_destroy( TaskInt, tasksList );
    tasksList = NULL;
  }
}

void Reset( int taskID )
{
  return;
}

bool HasError( int taskID )
{
  return false;
}

size_t GetMaxInputSamplesNumber( int taskID )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return 0;
  
  return 1;
}

size_t Read( int taskID, unsigned int channel, double* ref_value )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return 0;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  //DEBUG_PRINT( "reading channel %u from task %d (index %d of %d)", channel, taskID, taskIndex, kh_size( tasksList ) );
  
  if( channel >= task->channelsNumber ) return 0;
  
  if( !(task->isRunning) ) return 0;
  
  *ref_value = (double) task->inputValuesList[ channel ];
  
  return 1;
}

bool CheckInputChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  DEBUG_PRINT( "aquiring channel %u from task %d (index %d of %d)", channel, taskID, taskIndex, kh_size( tasksList ) );
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( channel > task->channelsNumber ) return false;
  
  //if( !(task->inputsUsedList[ channel ]) ) return false;
  
  //task->inputsUsedList[ channel ] = true;
  
  if( !(task->isRunning) ) task->threadID = Thread_Start( AsyncTransfer, task, THREAD_JOINABLE );
  
  return true;
}

bool IsOutputEnabled( int taskID )
{
  return true;
}

bool Write( int taskID, unsigned int channel, double value )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( !(task->isRunning) ) return false;
  
  if( channel > task->channelsNumber ) return false;
  
  task->outputValuesList[ channel ] = value;
  
  return true;
}

bool AcquireOutputChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  //DEBUG_PRINT( "aquiring channel %u from task %d", channel, taskID );
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( channel > task->channelsNumber ) return false;
  
  if( !(task->outputsUsedList[ channel ]) ) return false;
  
  task->outputsUsedList[ channel ] = true;
  
  if( !(task->isRunning) ) task->threadID = Thread_Start( AsyncTransfer, task, THREAD_JOINABLE );
  
  return true;
}

void ReleaseOutputChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( !(task->isRunning) ) return;
  
  if( channel > task->channelsNumber ) return;
  
  task->outputsUsedList[ channel ] = false; 
  
  (void) CheckTask( task );
}



static void* AsyncTransfer( void* callbackData )
{
  SignalIOTask task = (SignalIOTask) callbackData;
  
  int32 aquiredSamplesCount;
  
  task->isRunning = true;
  
  DEBUG_PRINT( "initializing read thread %lx", Thread_GetID() );
  
  while( task->isRunning )
  {
    int32 errorCode = DAQmxReadDigitalU16( task->misoReader, DAQmx_Val_Auto, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByScanNumber, 
                                           task->misoData, TRANSFER_BUFFER_LENGTH + 1, &aquiredSamplesCount, NULL );

    if( errorCode < 0 )
    {
      static char errorMessage[ DEBUG_MESSAGE_LENGTH ];
      DAQmxGetErrorString( errorCode, errorMessage, DEBUG_MESSAGE_LENGTH );
      fprintf( stderr, "reading error: %s\r", errorMessage );
    }
    else
    {
      for( size_t channel = 0; channel < task->channelsNumber; channel++ )
      {
        int32 inputChannelValue = 0; 
        for( int sampleIndex = 0; sampleIndex < TRANSFER_BUFFER_LENGTH; sampleIndex++ )
          inputChannelValue += ( ( task->misoData[ ( sampleIndex + 1 ) * task->channelsNumber + channel ] ) << ( TRANSFER_BUFFER_LENGTH - 1 - sampleIndex ) );
        int32 overflowsNumber = task->inputValuesList[ channel ] / INT16_MAX;
        if( task->inputValuesList[ channel ] < 0 ) overflowsNumber--;
        int32 newInputValue = overflowsNumber * INT16_MAX + inputChannelValue;
        if( task->inputValuesList[ channel ] - newInputValue > INT16_MAX / 2 ) overflowsNumber++;
        if( task->inputValuesList[ channel ] - newInputValue < -INT16_MAX / 2 ) overflowsNumber--;
        task->inputValuesList[ channel ] = overflowsNumber * INT16_MAX + inputChannelValue;
      }
    }
  }
  
  DEBUG_PRINT( "ending aquisition thread %x", Thread_GetID() );
  
  return NULL;
}

bool CheckTask( SignalIOTask task )
{
  bool isStillUsed = false;
  for( size_t channel = 0; channel < task->channelsNumber; channel++ )
  {
    if( task->inputsUsedList[ channel ] || task->outputsUsedList[ channel ] )
    {
      isStillUsed = true;
      break;
    }
  }
  
  if( !isStillUsed )
  {
    task->isRunning = false;
    if( task->threadID != THREAD_INVALID_HANDLE ) Thread_WaitExit( task->threadID, 5000 );
  }
  
  return isStillUsed;
}

SignalIOTask LoadTaskData( char* taskConfig )
{
  bool loadError = false;
  
  SignalIOTask newTask = (SignalIOTask) malloc( sizeof(SignalIOTaskData) );
  memset( newTask, 0, sizeof(SignalIOTask) );
  
  // National Instruments sucks. Seriously, their products aren't worth a pile of crap
  //
  // That said, for making that SPI interface work, the clock generator and chip selector tasks must be pulse generators
  // The clock period must be at most 1/16 of chip select low time in order to allow a complete 16-bit integer reading
  // The encoder read (MISO) task must be a single run 16 samples aquisition, using clock for timing and chip select falling edge
  // for triggering (that way, the reading will be restarted every time the slave is enabled)
  //
  // Clock and Chip Selector signals are shared among all devices, but each one has its own MISO digital line
  // That's not the standard way to build a SPI connection, but again, blame those NI bastards and their artificial limitations
  int32 clockGeneratorError = DAQmxLoadTask( strtok( taskConfig, " " ), &(newTask->clockGenerator) );
  int32 misoReaderError = DAQmxLoadTask( strtok( NULL, " " ), &(newTask->misoReader) );
  newTask->mosiWriter = NULL;
  int32 chipSelectorError = DAQmxLoadTask( strtok( NULL, " " ), &(newTask->chipSelector) );
  if( clockGeneratorError >= 0 && misoReaderError >= 0 && chipSelectorError >= 0 )
  {
    if( DAQmxGetTaskAttribute( newTask->misoReader, DAQmx_Task_NumChans, &(newTask->channelsNumber) ) >= 0 )
    {
      DEBUG_PRINT( "%u signal channels found", newTask->channelsNumber );
  
      newTask->inputsUsedList = (bool*) calloc( newTask->channelsNumber, sizeof(bool) );
      memset( newTask->inputsUsedList, 0, newTask->channelsNumber * sizeof(bool) );
      newTask->misoData = (uInt16*) calloc( newTask->channelsNumber * ( TRANSFER_BUFFER_LENGTH + 1 ), sizeof(uInt16) );
      newTask->inputValuesList = (int32*) calloc( newTask->channelsNumber, sizeof(int32) );
      
      newTask->outputsUsedList = (bool*) calloc( newTask->channelsNumber, sizeof(bool) );
      memset( newTask->outputsUsedList, 0, newTask->channelsNumber * sizeof(bool) );
      newTask->mosiData = (uInt16*) calloc( newTask->channelsNumber * ( TRANSFER_BUFFER_LENGTH + 1 ), sizeof(uInt16) );
      newTask->outputValuesList = (int32*) calloc( newTask->channelsNumber, sizeof(int32) );
  
      if( DAQmxStartTask( newTask->chipSelector ) < 0 || DAQmxStartTask( newTask->clockGenerator ) < 0 )
      {
        DEBUG_PRINT( "error starting task %s", taskConfig );
        loadError = true;
      }
      
      newTask->isRunning = false;
      newTask->threadID = THREAD_INVALID_HANDLE;
    }
    else 
    {
      //DEBUG_PRINT( "error getting task %s attribute", taskName );
      loadError = true;
    }
  }
  else 
  {
    //DEBUG_PRINT( "error loading task %s", taskName );
    loadError = true;
  }
  
  if( loadError )
  {
    UnloadTaskData( newTask );
    return NULL;
  }
  
  return newTask;
}

void UnloadTaskData( SignalIOTask task )
{
  if( task == NULL ) return;
  
  //DEBUG_PRINT( "ending task with handle %d", task->handle );

  DAQmxStopTask( task->clockGenerator );
  DAQmxStopTask( task->misoReader );
  DAQmxStopTask( task->chipSelector );
  
  DAQmxClearTask( task->clockGenerator );
  DAQmxClearTask( task->misoReader );
  DAQmxClearTask( task->chipSelector );

  if( task->inputsUsedList != NULL ) free( task->inputsUsedList );
  if( task->misoData != NULL ) free( task->misoData );
  if( task->inputValuesList != NULL ) free( task->inputValuesList );
  
  if( task->outputsUsedList != NULL ) free( task->outputsUsedList );
  if( task->mosiData != NULL ) free( task->mosiData );
  if( task->outputValuesList != NULL ) free( task->outputValuesList );
  
  free( task );
}
