#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

const unsigned CHECK_POINT_RADIUS = 600;
const unsigned POD_RADIUS = 400;
const float PI = 3.14159f;
const int MAX_ROTATION = 18;
const int SHIELD_BREAK_TURNS = 3;
const int TURNS_TO_CHECKPOINT = 100;
const float FRICTION = 0.85f;

#define SHOW_DEBUG 0

using namespace std;


struct World;

World* world = nullptr;


float RadiansToDegrees(float radians) 
{
    return radians*(180.0f/PI);
}

float DegreesToRadians(float degrees) 
{
    return degrees*(PI/180.0f);
}

int GetNormalizedAngleDegrees(int angle)
{
    if(angle < -180) angle += 360;
    if(angle > 180) angle -= 360;

    return angle;
}

int Clamp(int value, int min, int max)
{
    return std::min(std::max(min, value), max);
}

struct Vec2
{
    int x;
    int y;

    Vec2() : x(0), y(0) {}
    Vec2(int _x, int _y) : x(_x), y(_y) {}

    static Vec2 FromAngle(int degrees, int length)
    {
        float radians = DegreesToRadians(degrees);

        return Vec2(cos(radians)*length, sin(radians)*length);
    }

    Vec2 operator-(const Vec2& rhs) const
    {
        Vec2 res;
        res.x = x-rhs.x;
        res.y = y-rhs.y;

        return res;
    }

    Vec2 operator+(const Vec2& rhs) const
    {
        Vec2 res;
        res.x = x+rhs.x;
        res.y = y+rhs.y;

        return res;
    }

    float Length() const
    {
        return sqrtf(float(x*x+y*y));
    }

    void Scale(float value)
    {
        float len = Length();
        float mul = value/len;

        x = int(x*mul);
        y = int(y*mul);
    }

    Vec2 Scaled(float value)
    {
        Vec2 res = *this;

        res.Scale(value);

        return res;
    }

    int Dot(const Vec2& rhs) const
    {
        return x*rhs.x+y*rhs.y;
    }

    int GetAngle() const
    {
        float angle = x != 0 ? atan2(float(y), float(x)): 0.0f;
        return int(RadiansToDegrees(angle));
    }

    int GetAngleBetween(const Vec2& rhs) const
    {
        float angle0 = x != 0 ? atan2(float(y), float(x)) : 0.0f;
        float angle1 = rhs.x != 0 ? atan2(float(rhs.y), float(rhs.x)) : 0.0f;

        float angle = angle0-angle1;

        if(angle > PI) angle -=2.0f*PI;
        if(angle <-PI) angle += 2.0F*PI;

        return int(RadiansToDegrees(angle));
    }

    void Rotate(int degrees)
    {
        float radians = DegreesToRadians(float(degrees));
        float cs = cos(radians);
        float sn = sin(radians);

        float xaux = x*cs-y*sn;
        float yaux = x*sn+y*cs;

        x = int(xaux);
        y = int(yaux);
    }

    float Distance(const Vec2& rhs) const
    {
        return (rhs-*this).Length();
    }

    Vec2 operator*(float value) const
    {
        Vec2 res;
        res.x = int(float(x)*value);
        res.y = int(float(y)*value);

        return res;
    }

    Vec2& operator*=(float value) 
    {
        x = int(float(x)*value);
        y = int(float(y)*value);

        return *this;
    }

    bool operator==(const Vec2& rhs) const
    {
        return x == rhs.x && y == rhs.y;
    }

    friend ostream& operator<< (ostream& os, const Vec2& vec)
    {
        os << "(" << vec.x << ", " << vec.y << ")";
        return os;
    }
};


struct Pod
{
    enum ActionType
    {
        BRAKE_FOR_ROTATION = 0,
        BRAKE_FOR_ARRIVAL,
        BRAKE_FOR_COLLISION,
        TRY_COLLISION_BOOST,
        TRY_COLLISION_SHIELD,
        BOOST,
        ACCELERATE,        
        ACTION_COUNT
    };

    struct Action
    {
        ActionType type = ACTION_COUNT;
        Vec2 target;
        bool applyShield = false;
        bool applyInertia = false;

        Action() {}
        Action(ActionType tp, const Vec2& tg, bool sh, bool in) : type(tp), target(tg), applyShield(sh), applyInertia(in) {}
    };

    struct State
    {
        Vec2 pos;
        Vec2 speed;
        int  angle = 0;
        Vec2 prevTargetDir;
    };

    State state;
    int   nextCheckpoint = 0;
    float nextDistance = 0;
    bool  boostDone   = false;
    int   position    = 0;
    int   lap         = 0;
    bool  leadingRace = false;
    int   turnsLeft   = TURNS_TO_CHECKPOINT;

    void UpdateInput();
    void ComputeNextLeadAction(Action& action) const;
    void ComputeNextSecondaryAction(Action& action) const;
    void SendAction(const Action& action);

    static void PredictState(const State& current, State& next, const Vec2& target, int acc, bool applyInertia);
    static void PredictState(const State& current, State& next, const Action& action);

private:
    static const char* actionNames[ACTION_COUNT];

    int  ComputeAngleToTarget(const Vec2& target) const;
    bool TryDrift(int acceleration) const;
    void PredictInertia(Vec2& position, bool& checkPointPassed, int& breakTurns, int turnCount, bool waitOneTurn, int acceleartion) const;
    void UseShield(Action& action) const;
    Vec2 ComputeCollisionSpeed(const Vec2& pos0, const Vec2& speed0, bool shield0, 
                               const Vec2& pos1, const Vec2& speed1,bool shield1) const;


    static Vec2 ApplyInertia(const State& state, const Vec2& target);
};

const char* Pod::actionNames[ACTION_COUNT] = { "Brake for rotation", "Brake for arrival", "Brake for avoid collision", "Trying collision with boost", "Trying collision with shield", "Boost", "Acceleration" };

struct World
{
    int     lapCount = 0;
    int     checkpointCount = 0;
    Vec2*   checkPoints = nullptr;
    Pod     player[2];
    Pod     opponent[2];

    ~World();

    void Init();
    void Update();
private:
    static void UpdateLeader(Pod& p0, Pod& p1);
    void AvoidFriendCollision(const Pod::Action& action0, const Pod::State& state0, 
                              Pod::Action& action1, const Pod::State& state1) const;
};


///////////////////////////////////////////////////
//  POD
///////////////////////////////////////////////////

void Pod::UpdateInput()
{
    int prevCheckpoint = nextCheckpoint;
    cin >> state.pos.x >> state.pos.y; 
    cin >> state.speed.x >> state.speed.y; 
    cin >> state.angle;
    cin >> nextCheckpoint;

    // Systems is one turn delayed
    if((world->checkPoints[nextCheckpoint]-state.pos).Length() < CHECK_POINT_RADIUS)
        ++nextCheckpoint;

    if(nextCheckpoint != prevCheckpoint)
        turnsLeft = TURNS_TO_CHECKPOINT;
    else
        --turnsLeft;

    if(nextCheckpoint == 0 && prevCheckpoint == world->checkpointCount-1)
        ++lap;    

    nextDistance = (state.pos-world->checkPoints[nextCheckpoint]).Length();

    cin.ignore();   
}

int Pod::ComputeAngleToTarget(const Vec2& target) const
{
    int diffAngle = (target-state.pos).GetAngle()-state.angle;

    return GetNormalizedAngleDegrees(diffAngle);
}

void Pod::PredictState(const State& current, State& next, const Action& action)
{
    int acceleration = 0;

    if(action.type == BOOST || action.type == TRY_COLLISION_BOOST)
        acceleration = 120;
    else if(action.type == ACCELERATE)
        acceleration = 100;

    PredictState(current, next, action.target, acceleration, action.applyInertia);
}

void Pod::PredictState(const State& current, State& next, const Vec2& target, int acc, bool applyInertia)
{
    Vec2 realTarget = target;
    if(applyInertia)
    {
        realTarget = ApplyInertia(current, target);
    }

    Vec2 targetDir = (realTarget-current.pos);

    int angleDiff = Clamp(GetNormalizedAngleDegrees(targetDir.GetAngle()-current.angle), -MAX_ROTATION, MAX_ROTATION);

    next.angle = GetNormalizedAngleDegrees(current.angle+angleDiff);

    // Predict speed
    Vec2 acceleration = Vec2::FromAngle(next.angle, acc);
    
    next.speed = current.speed+acceleration;
    next.pos   = current.pos+next.speed;
    next.speed *= FRICTION;
    next.prevTargetDir = (target-current.pos);
}

void Pod::PredictInertia(Vec2& position, bool& checkPointPassed, int& breakTurns, int turnCount, bool waitOneTurn, int acceleration) const
{
    const Vec2& currentPos = world->checkPoints[nextCheckpoint]; 
    const Vec2& nextPos    = world->checkPoints[(nextCheckpoint+1)%world->checkpointCount]; 

    State nextState;
    State prevState = state;

    checkPointPassed = false;
    breakTurns = 0;

    for(int i=0; i< turnCount; ++i)
    {
        if(waitOneTurn && i==0)
        {
            PredictState(prevState, nextState, currentPos,  100, true);
        }
        else
        {
            PredictState(prevState, nextState, nextPos, checkPointPassed ? 100 : acceleration, true);
        }

        if(!waitOneTurn && !checkPointPassed)
        {
            ++breakTurns;
        }

        prevState = nextState;

        checkPointPassed = checkPointPassed || (nextState.pos-currentPos).Length() < CHECK_POINT_RADIUS;
    }

    position = nextState.pos;
}

bool Pod::TryDrift(int acceleration) const
{
    const int TURN_PREDICTION = 6;

    // last checkpoint
    if(lap == world->lapCount && nextCheckpoint == 0)
    {
        return false;
    }

    Vec2 driftPos;
    bool driftPassed;
    int driftBreaks;
    PredictInertia(driftPos, driftPassed, driftBreaks, TURN_PREDICTION, false, acceleration);

    if(driftPassed)
    {
        const Vec2& nextPos = world->checkPoints[(nextCheckpoint+1)%world->checkpointCount]; 

        Vec2 noDriftPos;
        bool noDriftPassed;
        int noDriftBreaks;
        PredictInertia(noDriftPos, noDriftPassed, noDriftBreaks, TURN_PREDICTION, true, acceleration);

        if(!noDriftPassed || driftPos.Distance(nextPos) < noDriftPos.Distance(nextPos))
        {
            return true;
        }
    }

    return false;
}

void Pod::ComputeNextSecondaryAction(Action& action) const
{
    if(turnsLeft < TURNS_TO_CHECKPOINT/6)
    {
        ComputeNextLeadAction(action);
    }
    else
    {
        Pod* oppLead = world->opponent[0].leadingRace ? &world->opponent[0] : &world->opponent[1];
        Pod::State& oppState = oppLead->state;


        Vec2 target;        

        Vec2 oppDir = world->checkPoints[oppLead->nextCheckpoint]-oppState.pos;

        // In front opponent
        if(abs(oppDir.GetAngleBetween(state.pos-oppState.pos)) < 45)
        {
            Pod::State oppNext;
            PredictState(oppState, oppNext, oppState.pos+oppState.speed, 100, false);
            target = oppNext.pos;
        }
        else
        {
            target = world->checkPoints[oppLead->nextCheckpoint];      
            Vec2 nextTarget = world->checkPoints[(oppLead->nextCheckpoint+1)%world->checkpointCount];
            int angle = (state.pos-target).GetAngleBetween(nextTarget-target);         
            
            // Not In front next target        
            if(abs(angle) > 25)
            {
                target = nextTarget;
            }
        }        

        int angleToTarget = ComputeAngleToTarget(target);
        if(angleToTarget > 90 || angleToTarget < -90)
        {
            action = Action(BRAKE_FOR_ROTATION, target, false, false);
        }        
        else
        {
            action = Action(ACCELERATE, target, false, true);
        }   

        UseShield(action);
    }
}

void Pod::ComputeNextLeadAction(Action& action) const
{
    const Vec2& currentCheckPointPos = world->checkPoints[nextCheckpoint]; 
    const Vec2& nextCheckPointPos = world->checkPoints[(nextCheckpoint+1)%world->checkpointCount]; 

    Vec2 target = currentCheckPointPos;
    bool driftWithShield = false;

    int angleToTarget = ComputeAngleToTarget(target);

    State prediction;
    PredictState(state, prediction, target, 100, true);

    if(angleToTarget > 90 || angleToTarget < -90)
    {
        action = Action(BRAKE_FOR_ROTATION, target, false, false);
    }        
    else if(TryDrift(0))
    {
        action = Action(BRAKE_FOR_ARRIVAL, nextCheckPointPos, false, true);
    }     
    else 
    {          
        float distance = (target-state.pos).Length();
        if(!boostDone && angleToTarget == 0 && distance > 3000)
        {
            action = Action(BOOST, target, false, false);
        }
        else
        {
            if(TryDrift(100))
            {
                action = Action(ACCELERATE, nextCheckPointPos, false, true);            
            }
            else
            {
                action = Action(ACCELERATE, target, false, true);            
            }                
        }
    }   

    UseShield(action);
}

Vec2 Pod::ApplyInertia(const State& state, const Vec2& target) 
{
    int inertiaAngle = state.speed.GetAngleBetween(state.prevTargetDir);
    int compensate = std::min(std::max(inertiaAngle, -MAX_ROTATION), MAX_ROTATION);

    Vec2 targetDir = target-state.pos;
    targetDir.Rotate(-compensate);

    return state.pos+targetDir;
}

void Pod::SendAction(const Action& action)
{
#if SHOW_DEBUG
    cerr << actionNames[action.type] << endl;
#endif

    Vec2 realTarget = action.target;

    // compensate inertia
    if(action.applyInertia) 
        realTarget = ApplyInertia(state, action.target);
 
    int acceleration = 0;

    switch(action.type)
    {
        case BRAKE_FOR_ROTATION: 
        case BRAKE_FOR_ARRIVAL: 
        case BRAKE_FOR_COLLISION:
            cout << realTarget.x << " " << realTarget.y << (action.applyShield ? " SHIELD" : " 0") << endl;            
            break;
        case BOOST: 
        case TRY_COLLISION_BOOST: 
            cout << realTarget.x << " " << realTarget.y << " BOOST" << endl;
            boostDone = true;
            acceleration = 120;
            break;
        case TRY_COLLISION_SHIELD:
            cout << realTarget.x << " " << realTarget.y << " SHIELD" << endl;
            break;
        case ACCELERATE:
        default:            
            acceleration = 100;
            cout << realTarget.x << " " << realTarget.y << " 100" << endl;
            break;
    }

    state.prevTargetDir  = (action.target-state.pos);    
}

Vec2 Pod::ComputeCollisionSpeed(const Vec2& pos0, const Vec2& speed0, bool shield0, 
                                  const Vec2& pos1, const Vec2& speed1,bool shield1) const
{
    // Compute elastic collision (note: positions are approximated)
    Vec2 posDiff = pos0-pos1;
    float posDist = posDiff.Dot(posDiff);

    float m1 = (shield0 ? 10.0f : 1.0f);
    float m2 = (shield1 ? 10.0f : 1.0f);

    float massProd = 2.0f*m2/(m1+m2);

    return  speed0-posDiff*(massProd*(speed0-speed1).Dot(posDiff)/posDist);
}

void Pod::UseShield(Action& action) const 
{
    Pod::State next;
    Pod::PredictState(state, next, action);    

    for(int i=0; i< 2; ++i)
    {
        Pod::State oppNext;
        const Pod::State& oppState = world->opponent[i].state;
        Pod::PredictState(oppState, oppNext, oppState.pos+oppState.speed, 100, false);

        if((oppNext.pos-next.pos).Length() < POD_RADIUS*2)  
        {            
            Vec2 v1 = ComputeCollisionSpeed(next.pos, next.speed, false, oppNext.pos, oppNext.speed, true);

            if(abs(v1.GetAngleBetween(next.speed)) > 90)
            {
                action.type = Pod::BRAKE_FOR_COLLISION;
                action.applyShield = true;
            }
        }
    }
}


///////////////////////////////////////////////////
//  WORLD
///////////////////////////////////////////////////

World::~World()
{
    delete [] checkPoints;
}


void World::Init()
{
    cin >> lapCount; cin.ignore();
    cin >> checkpointCount; cin.ignore();

    checkPoints = new Vec2[checkpointCount];

    for(int i=0; i< checkpointCount; ++i)
    {
        cin >> checkPoints[i].x >> checkPoints[i].y; cin.ignore();
    }        
}

void World::AvoidFriendCollision(const Pod::Action& action0, const Pod::State& state0, 
                           Pod::Action& action1, const Pod::State& state1) const
{
    Pod::State current0 = state0;
    Pod::State current1 = state1;
    Pod::State next0, next1;

    for(int i=0; i< 8; ++i)
    {
        Pod::PredictState(current0, next0, action0);
        Pod::PredictState(current1, next1, action1);

        if((next0.pos-next1.pos).Length() < POD_RADIUS*2)
        {            
            if(abs(next0.speed.GetAngleBetween(state1.pos-state0.pos)) > 90)
            {
                action1.type = Pod::BRAKE_FOR_COLLISION;
            }
            else
            {
                action1.type = Pod::ACCELERATE;
                action1.applyShield = false;
                action1.target = state1.pos+state0.speed;
            }
            break;
        }

        current0 = next0;
        current1 = next1;
    }
}

void World::Update()
{
    for(int i=0; i <2; ++i) 
    {
        player[i].UpdateInput();
    }

    UpdateLeader(player[0], player[1]);

    for(int i=0; i <2; ++i) 
    {
        opponent[i].UpdateInput();
    }

    UpdateLeader(opponent[0], opponent[1]);

    Pod::Action action0, action1;
    if(player[0].leadingRace)
    {
        player[0].ComputeNextLeadAction(action0);
        player[1].ComputeNextSecondaryAction(action1);
    }
    else
    {
        player[1].ComputeNextLeadAction(action1);
        player[0].ComputeNextSecondaryAction(action0);        
    }
    

    if(player[0].leadingRace)
    {
        AvoidFriendCollision(action0, player[0].state, action1, player[1].state);
    }
    else
    {
        AvoidFriendCollision(action1, player[1].state, action0, player[0].state);
    }

    player[0].SendAction(action0);
    player[1].SendAction(action1);
}

void World::UpdateLeader(Pod& p0, Pod& p1)
{
    bool firstLeading = p0.lap > p1.lap;
    firstLeading = firstLeading || (p0.lap == p1.lap && 
                p0.nextCheckpoint > p1.nextCheckpoint);
    firstLeading = firstLeading || (p0.lap == p1.lap && 
                p0.nextCheckpoint ==  p1.nextCheckpoint &&
                p0.nextDistance < p1.nextDistance);

    p0.leadingRace = firstLeading;
    p1.leadingRace = !firstLeading;
}

int main()
{
    world = new World;

    world->Init();

    // game loop
    while (1) 
    {
        world->Update();
    }
    delete world;
}

