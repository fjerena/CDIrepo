#Project Backlog

Pin A8 was defined as output because this first version I will not use the sensor VRS2 
Pin A15 is connected to VRS1

I should organize/clean main branch as much as I can


Requirements to create more efficient and generalistic Scheduller to use in my CDI project:

Build a code that I can create, delete, edit processes in real time to be treated by core/uC one by one at once or periodically in a smart way 

-Set a time to be executed (time in ms)
-Priority (256 priority levels)
-Be able to call a function that I going to pass as a parameter to my function (Or maybe execute a chunk of code/Program)
-Capacity to block temporary the process execution (enable/disable)
-Service to list process execution queue: order of execution (combination between time and priority) and name/ID

The list of process will be organized by time to execute and after priority
-Always when you change something in the list you will need to re-organize the list  

Things that I going need to create:
-function to call another functions after to receive a pointer function
-Efficient algorithm to order the list fast

create_process(name/ID, execution, priority, memory_reservation
name/ID: Some unique/exclusive identification for a new process 
execution: periodic, unique
priority: 0 (most important) - 255 (less important)
exec_allowed: true or false (this is a status, always that you create a new process it will start as "true" and if you want you can change to "false" to block the execution) 
memory_reservation: I don know how to manage this part and if I need it

The processes can assume two different status:

Schedulled
Executing

and 

if the execution is allowed

**** Just to think about it and organize the thoughts ****

Process has basically:

Properties
name/ID: give a unique identification label
execution: shows if will be executed at once or periodically
priority: it defines which process will be executed when two or more processes has the same target time
exec_allowed: it defines if the process is allowed to be executed  
status: shows if the process is Schedulled or is Running 
ExecutionTarget: will appoint to a function address that it will executed when the process it runs

*After its done, should the destroyed or re-scheduled (periodic process)

block_process(ID)
exec_allowed: false

release_process(ID)
exec_allowed: true
 
change_priority(ID,new_priority)

change_execution(ID,execution)

destroy_process(ID)

Table example:

Order	Name		Time to Execute	Priority 	Pointing to (chunk of code)
0	Process1	1000			2		*func2
1	Process6	1000			2		*func0
2	Process3	1000			3		0x0001
3	Process2	2780			0		*func1
4	Process5	3000			0		*func3
5	Process4	5000			1		0x5500

*To create the sort algorithm should be really good to implement one better then bubble sort (lower performance)

Create a array with 8 bits where I going to include the ID in correct order:

To create the ExecutionList I need to order two times (2x) the table, the first one, focused to Time2Execute and the second one by Priority
ExecutionList going to store the "Name" (pointer) to a specific process, and the struct_Process will have: Order, Name, Time2Execute, Priority, exec_allowed, Pointer for a specific func or an address, status

enum enumProcess = {"SCHEDULLED", "RUNNING"};

struct Process_Mgt
{
	char Name[255];         //Maybe a string or just a number ID
	uint32_t Time2Execute;  //Should filled with the current time + the period expected to execute the process
	uint8_t Priority;       //0-255 (high to Low)
	void (*func)(void);	 //ExecutionTarget;        //Pointing to some function or chunk of code
	uint8_t Exec_allowed;   //True or False
	enumProcess Status;     //shows if the process is Schedulled or is Running 
}ExecutionList;

I need to create two data structure manager:

-List (with all basic functionality, push, pull, reset)  //First in, first out

struct List
{
	uint16_t NextIn;
	uint16_t NextOut;
	uint8_t ListArray[];  //256 positions
}My_List;

void Start_List(List *ListName)
{
	uint16_t i;
	
	*ListArray->NextIn=0;
	*ListArray->NextOut=0;
	
	for(i=0;i<256u;i++)
	{
		*ListArray->ListArray[i]=0;
	}
}

int8_t Push_Element_to_List(List *ListName, uint8_t value)
{
	if(*ListName->NextIn<256u)
	{
		*ListName->ListArray[*ListName->NextIn]=value;
		*ListName->NextIn++;	
		return 0;  //The value already pushed
	}
	else
	{
		return -1; //List is Full
	}	
}


//Pay attention because here I just can return values up to +255 (or maybe +256)
int16_t Pull_Element_from_List(List *ListName)
{
	if(*ListName->NextIn!=*ListName->NextOut)
	{
		return *ListArray->ListArray[NextOut];	
		*ListArray->NextOut++;
	}
	else
	{
		return -1; //Is empty
	}
}

-Stack (with all basic functionality, push, pull, reset)  //First in, last out

void Start_Stack(Stack *StackName)
{
	*StackArray->NextIn=0;
	*StackArray->NextOut=0;
	
	for(i=0;i<256u;i++)
	{
		*StackArray->StackArray[i]=0;
	}
}

int8_t Push_Element_to_Stack(Stack *StackName, uint8_t value)
{
	if(*StackName->NextIn<256u)
	{
		*StackName->StackArray[*StackName->NextIn]=value;
		*StackName->NextIn++;		
	}
	else
	{
		return -1;//List is Full
	}	
}

int8_t Pull_Element_from_Stack(Stack *StackName)
{
	if(*StackName->NextIn!=*StackName->NextOut)
	{
		return *StackArray->StackArray[NextOut];	
		*StackArray->NextOut++;
	}
	else
	{
		return -1; //Is empty
	}
}

Order;	//Filled after ExecutionList was analysed

ExecutionList[255];  //List to control the existent processes
Elements_in_the_list//Create some variable to count the elements in this list

for(i=0,i<last_register,i++)
{
	if(struct_Process[ExecutionList[i]].time2execute==CurrentTime)
	{
		//Execute a function or code
	}
}

Process in theory is a program, a program has variables, functions (but I don know how to create these entities by code/run time), I just know to call functions that I already 
declared in my code

Questions;

-How to create a process (chunk of code) to be executed by this scheduller that I thinking to create?

Allocate new variables and functions on RAM memory (I need to study about this to have more clarity), how to create a new program inside another program...



Re-thinking about the transmission solution (maybe create some code to treat communication in a different level), I need to analyse the real needs to do it because I didn't have time to thinking
about it, but the main idea is create a sw as more generalistic that I can

