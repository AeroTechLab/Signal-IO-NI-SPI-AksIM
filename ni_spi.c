////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2019 Leonardo Consoni <leonardojc@protonmail.com>      //
//                                                                            //
//  This file is part of Signal-IO-NISPI.                                     //
//                                                                            //
//  Signal-IO-NISPI is free software: you can redistribute it and/or modify   //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  Signal-IO-NISPI is distributed in the hope that it will be useful,        //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with Signal-IO-NISPI. If not, see <http://www.gnu.org/licenses/>.   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

// PLEASE READ BEFORE TRYING TO USE THIS CODE!!
//
// National Instruments sucks. Seriously, their products aren't worth a pile of crap
//
// That said, for making that SPI interface work, the clock generator and chip selector tasks must be pulse generators
// The clock period must be at most 1/16 of chip select low time in order to allow a complete 16-bit integer reading
// And you must be able to perform many reads inside a single control time step, as an average filter is applied
// The encoder read (MISO) task must be a single run 16 samples aquisition, using clock for timing and chip select falling edge
// for triggering (that way, the reading will be restarted every time the slave is enabled)
//
// Clock and Chip Selector signals are shared among all devices, but each one has its own MISO digital line
// That's not the standard way to build a SPI connection, but again, blame those NI bastards and their artificial limitations


#include "signal_io.h"

#include <NIDAQmx.h>

#include "threads/threads.h"
#include "threads/khash.h"

#include "debug/data_logging.h"

#define DEBUG_MESSAGE_LENGTH 256

#define INT16_MAX 65536

#define CHANNELS_MAX_NUMBER 8
#define READ_SAMPLES_NUMBER 5

const size_t TRANSFER_BUFFER_LENGTH = 16;

typedef struct _SignalIOTaskData
{
  TaskHandle clockGenerator;
  TaskHandle misoReader;
  TaskHandle chipSelector;
  Thread threadID;
  volatile bool isRunning;
  uInt16* misoData;
  int32* inputValuesList;
  bool* inputsUsedList;
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

long int InitDevice( const char* taskConfig )
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
  
  return (long int) kh_key( tasksList, newTaskIndex );
}

void EndDevice( long int taskID )
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

void Reset( long int taskID )
{
  return;
}

bool HasError( long int taskID )
{
  return false;
}

size_t GetMaxInputSamplesNumber( long int taskID )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return 0;
  
  return 1;
}

size_t Read( long int taskID, unsigned int channel, double* ref_value )
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

bool CheckInputChannel( long int taskID, unsigned int channel )
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

bool IsOutputEnabled( long int taskID )
{
  return false;
}

bool Write( long int taskID, unsigned int channel, double value )
{
  return false;
}

bool AcquireOutputChannel( long int taskID, unsigned int channel )
{
  return false;
}

void ReleaseOutputChannel( long int taskID, unsigned int channel )
{
  return;
}



static void* AsyncTransfer( void* callbackData )
{
  SignalIOTask task = (SignalIOTask) callbackData;
  
  int32 aquiredSamplesCount;
  
  int32 channelSamplesTable[ CHANNELS_MAX_NUMBER ][ READ_SAMPLES_NUMBER ] = { 0 };
  size_t channelReadsCount = 0;
  
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
        for( int bitIndex = 0; bitIndex < TRANSFER_BUFFER_LENGTH; bitIndex++ )
          inputChannelValue += ( ( task->misoData[ ( bitIndex + 1 ) * task->channelsNumber + channel ] ) << ( TRANSFER_BUFFER_LENGTH - 1 - bitIndex ) );
        channelSamplesTable[ channel ][ channelReadsCount ] = inputChannelValue;
      }
      
      channelReadsCount++;
    }
    
    if( channelReadsCount >= READ_SAMPLES_NUMBER )
    {
      for( size_t channel = 0; channel < task->channelsNumber; channel++ )
      {
        int32 inputChannelAverageValue = 0;
        for( int sampleIndex = 0; sampleIndex < channelReadsCount; sampleIndex++ )
            inputChannelAverageValue += channelSamplesTable[ channel ][ sampleIndex ] / channelReadsCount;
        int32 overflowsNumber = task->inputValuesList[ channel ] / INT16_MAX;
        if( task->inputValuesList[ channel ] < 0 ) overflowsNumber--;
        int32 newInputValue = overflowsNumber * INT16_MAX + inputChannelAverageValue;
        if( ( task->inputValuesList[ channel ] - newInputValue ) > ( INT16_MAX / 2 ) ) overflowsNumber++;
        if( ( task->inputValuesList[ channel ] - newInputValue ) < ( -INT16_MAX / 2 ) ) overflowsNumber--;
        task->inputValuesList[ channel ] = overflowsNumber * INT16_MAX + inputChannelAverageValue;
        //fprintf( stderr, "aquired %d samples with value %d (overflows=%d, corrected=%d)\r", aquiredSamplesCount, inputChannelValue, overflowsNumber, task->inputValuesList[ channel ] );
      }
      
      channelReadsCount = 0;
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
    if( task->inputsUsedList[ channel ] )
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
  
  int32 clockGeneratorError = DAQmxLoadTask( strtok( taskConfig, " " ), &(newTask->clockGenerator) );
  int32 misoReaderError = DAQmxLoadTask( strtok( NULL, " " ), &(newTask->misoReader) );
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
  
  free( task );
}
