#include <napi.h>
#include "physics.h"

namespace Physics {

// Wrapper class for the Physics Engine
class PhysicsWrapper : public Napi::ObjectWrap<PhysicsWrapper> {
public:
    static Napi::Object Init(Napi::Env env, Napi::Object exports) {
        Napi::Function func = DefineClass(env, "Engine", {
            InstanceMethod("initialize", &PhysicsWrapper::Initialize),
            InstanceMethod("update", &PhysicsWrapper::Update),
            InstanceMethod("createBox", &PhysicsWrapper::CreateBox),
            InstanceMethod("createSphere", &PhysicsWrapper::CreateSphere),
            InstanceMethod("createPlane", &PhysicsWrapper::CreatePlane),
            InstanceMethod("createHingeConstraint", &PhysicsWrapper::CreateHingeConstraint),
            InstanceMethod("setHingeConstraintRotation", &PhysicsWrapper::SetHingeConstraintRotation),
            InstanceMethod("toggleDebugDraw", &PhysicsWrapper::ToggleDebugDraw),
            InstanceMethod("toggleColliders", &PhysicsWrapper::ToggleColliders),
            InstanceMethod("toggleGrid", &PhysicsWrapper::ToggleGrid),
            InstanceMethod("resetScene", &PhysicsWrapper::ResetScene),
            InstanceMethod("getDebugDrawData", &PhysicsWrapper::GetDebugDrawData),
            InstanceMethod("getAverageFPS", &PhysicsWrapper::GetAverageFPS),
            InstanceMethod("getWorld", &PhysicsWrapper::GetWorld)
        });

        Napi::FunctionReference* constructor = new Napi::FunctionReference();
        *constructor = Napi::Persistent(func);
        env.SetInstanceData(constructor);
        exports.Set("Engine", func);
        return exports;
    }

    PhysicsWrapper(const Napi::CallbackInfo& info) : Napi::ObjectWrap<PhysicsWrapper>(info) {
        engine = new Engine();
    }

    ~PhysicsWrapper() {
        delete engine;
    }

private:
    Engine* engine;

    Napi::Value Initialize(const Napi::CallbackInfo& info) {
        engine->Initialize();
        return info.Env().Undefined();
    }

    Napi::Value Update(const Napi::CallbackInfo& info) {
        float deltaTime = info[0].As<Napi::Number>().FloatValue();
        engine->Update(deltaTime);
        return info.Env().Undefined();
    }

    Napi::Value CreateBox(const Napi::CallbackInfo& info) {
        Napi::Object pos = info[0].As<Napi::Object>();
        Napi::Object size = info[1].As<Napi::Object>();
        float mass = info[2].As<Napi::Number>().FloatValue();

        Vector3 position(
            pos.Get("x").As<Napi::Number>().FloatValue(),
            pos.Get("y").As<Napi::Number>().FloatValue(),
            pos.Get("z").As<Napi::Number>().FloatValue()
        );

        Vector3 dimensions(
            size.Get("x").As<Napi::Number>().FloatValue(),
            size.Get("y").As<Napi::Number>().FloatValue(),
            size.Get("z").As<Napi::Number>().FloatValue()
        );

        RigidBody* body = engine->CreateBox(position, dimensions, mass);
        return Napi::Number::New(info.Env(), reinterpret_cast<uintptr_t>(body));
    }

    Napi::Value CreateSphere(const Napi::CallbackInfo& info) {
        Napi::Object pos = info[0].As<Napi::Object>();
        float radius = info[1].As<Napi::Number>().FloatValue();
        float mass = info[2].As<Napi::Number>().FloatValue();

        Vector3 position(
            pos.Get("x").As<Napi::Number>().FloatValue(),
            pos.Get("y").As<Napi::Number>().FloatValue(),
            pos.Get("z").As<Napi::Number>().FloatValue()
        );

        RigidBody* body = engine->CreateSphere(position, radius, mass);
        return Napi::Number::New(info.Env(), reinterpret_cast<uintptr_t>(body));
    }

    Napi::Value CreatePlane(const Napi::CallbackInfo& info) {
        Napi::Object normal = info[0].As<Napi::Object>();
        float distance = info[1].As<Napi::Number>().FloatValue();
        float mass = info[2].As<Napi::Number>().FloatValue();

        Vector3 planeNormal(
            normal.Get("x").As<Napi::Number>().FloatValue(),
            normal.Get("y").As<Napi::Number>().FloatValue(),
            normal.Get("z").As<Napi::Number>().FloatValue()
        );

        RigidBody* body = engine->CreatePlane(planeNormal, distance, mass);
        return Napi::Number::New(info.Env(), reinterpret_cast<uintptr_t>(body));
    }

    Napi::Value CreateHingeConstraint(const Napi::CallbackInfo& info) {
        Napi::Object pivot = info[0].As<Napi::Object>();
        Napi::Object axis = info[1].As<Napi::Object>();
        float angularVelocity = info[2].As<Napi::Number>().FloatValue();
        bool isRotating = info[3].As<Napi::Boolean>().Value();

        Vector3 pivotPoint(
            pivot.Get("x").As<Napi::Number>().FloatValue(),
            pivot.Get("y").As<Napi::Number>().FloatValue(),
            pivot.Get("z").As<Napi::Number>().FloatValue()
        );

        Vector3 rotationAxis(
            axis.Get("x").As<Napi::Number>().FloatValue(),
            axis.Get("y").As<Napi::Number>().FloatValue(),
            axis.Get("z").As<Napi::Number>().FloatValue()
        );

        HingeConstraint* constraint = engine->CreateHingeConstraint(pivotPoint, rotationAxis, angularVelocity, isRotating);
        return Napi::Number::New(info.Env(), reinterpret_cast<uintptr_t>(constraint));
    }

    Napi::Value SetHingeConstraintRotation(const Napi::CallbackInfo& info) {
        int constraintId = info[0].As<Napi::Number>().Int32Value();
        float angle = info[1].As<Napi::Number>().FloatValue();
        engine->SetHingeConstraintRotation(constraintId, angle);
        return info.Env().Undefined();
    }

    Napi::Value ToggleDebugDraw(const Napi::CallbackInfo& info) {
        engine->ToggleDebugDraw();
        return info.Env().Undefined();
    }

    Napi::Value ToggleColliders(const Napi::CallbackInfo& info) {
        engine->ToggleColliders();
        return info.Env().Undefined();
    }

    Napi::Value ToggleGrid(const Napi::CallbackInfo& info) {
        engine->ToggleGrid();
        return info.Env().Undefined();
    }

    Napi::Value ResetScene(const Napi::CallbackInfo& info) {
        engine->ResetScene();
        return info.Env().Undefined();
    }

    Napi::Value GetDebugDrawData(const Napi::CallbackInfo& info) {
        const DebugDrawData& data = engine->GetDebugDrawData();
        Napi::Object result = Napi::Object::New(info.Env());

        // Convert lines
        Napi::Array lines = Napi::Array::New(info.Env());
        for (size_t i = 0; i < data.lines.size(); i++) {
            Napi::Object line = Napi::Object::New(info.Env());
            
            Napi::Object start = Napi::Object::New(info.Env());
            start.Set("x", data.lines[i].start.x);
            start.Set("y", data.lines[i].start.y);
            start.Set("z", data.lines[i].start.z);
            
            Napi::Object end = Napi::Object::New(info.Env());
            end.Set("x", data.lines[i].end.x);
            end.Set("y", data.lines[i].end.y);
            end.Set("z", data.lines[i].end.z);
            
            Napi::Object color = Napi::Object::New(info.Env());
            color.Set("x", data.lines[i].color.x);
            color.Set("y", data.lines[i].color.y);
            color.Set("z", data.lines[i].color.z);
            
            line.Set("start", start);
            line.Set("end", end);
            line.Set("color", color);
            
            lines.Set(i, line);
        }
        result.Set("lines", lines);

        // Convert points
        Napi::Array points = Napi::Array::New(info.Env());
        for (size_t i = 0; i < data.points.size(); i++) {
            Napi::Object point = Napi::Object::New(info.Env());
            
            Napi::Object position = Napi::Object::New(info.Env());
            position.Set("x", data.points[i].position.x);
            position.Set("y", data.points[i].position.y);
            position.Set("z", data.points[i].position.z);
            
            Napi::Object color = Napi::Object::New(info.Env());
            color.Set("x", data.points[i].color.x);
            color.Set("y", data.points[i].color.y);
            color.Set("z", data.points[i].color.z);
            
            point.Set("position", position);
            point.Set("color", color);
            point.Set("size", data.points[i].size);
            
            points.Set(i, point);
        }
        result.Set("points", points);

        return result;
    }

    Napi::Value GetAverageFPS(const Napi::CallbackInfo& info) {
        return Napi::Number::New(info.Env(), engine->GetAverageFPS());
    }

    Napi::Value GetWorld(const Napi::CallbackInfo& info) {
        return Napi::Number::New(info.Env(), reinterpret_cast<uintptr_t>(engine->GetWorld()));
    }
};

Napi::Object Init(Napi::Env env, Napi::Object exports) {
    return PhysicsWrapper::Init(env, exports);
}

NODE_API_MODULE(physics, Init)

} // namespace Physics 