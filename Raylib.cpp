#include "raylib.h"
#include "raymath.h" // For vector math functions
#include <vector>
#include <cmath>
#include <deque>      // For trails
#include <numeric>    // For std::accumulate
#include <iostream>   // For potential debugging
#include <iomanip>    // For formatting output floats
#include <sstream>    // For formatting text

// --- Constants and Global Variables ---
const int screenWidth = 1000; // Increased size slightly
const int screenHeight = 750;
const float G = 6.674e-11f * 1e12f; // Adjusted G for visual scale (Needs tuning!)
const float softingFactor = 5.1f;   // Softening factor (Needs tuning!)
const int MAX_TRAIL_LENGTH = 200;   // Max points in trail

struct Body {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration; // Store current acceleration
    float mass;
    float radius;
    Color color;
    std::deque<Vector2> trail;
};

// Using std::vector is slightly more C++ idiomatic, though array works fine for fixed size
std::vector<Body> bodies(3);
std::vector<Body> initialBodies(3); // Store initial state for restart

bool paused = false;
int selectedBody = 0; // Keep track of selected body
double simulationTime = 0.0;
float currentTimeStep = 0.05f; // Smaller timestep often needed for RK4 stability
float timeScale = 1.0f;        // To speed up/slow down simulation visually

Camera2D camera = { 0 };

// --- RK4 Data Structures ---
struct State {
    Vector2 position;
    Vector2 velocity;
};

struct Derivative {
    Vector2 dpdt; // Derivative of position (velocity)
    Vector2 dvdt; // Derivative of velocity (acceleration)
};

// --- Function Declarations ---
void InitializeBodies();
void InitializeCamera();
void CalculateSystemAccelerations(const std::vector<State>& currentStates, std::vector<Derivative>& outputDerivatives);
Vector2 CalculateGravity(const Body& body1, const Body& body2); // Uses Body struct
Vector2 CalculateGravityFromState(const State& state1, float mass1, const State& state2, float mass2); // Uses State struct
void UpdateBodiesRK4(float dt);
void UpdateCamera();
void DrawSimulation();
void DrawUI();
void DrawTrails();
void HandleInput();
void RestartSimulation();
void AddToTrail(Body& body);
State StateAddDt(const State& s, const Derivative& d, float dt);
double CalculateTotalKineticEnergy();
double CalculateTotalPotentialEnergy();
const char* ColorToString(Color c);

// --- Main Function ---
int main() {
    InitWindow(screenWidth, screenHeight, "Three-Body Problem (RK4) - Raylib");
    SetTargetFPS(60);

    InitializeBodies();
    InitializeCamera();
    for (int i = 0; i < 3; ++i) {
        initialBodies[i] = bodies[i]; // Store initial state
    }

    while (!WindowShouldClose()) {
        HandleInput();
        UpdateCamera();

        if (!paused) {
            // --- Simulation Step ---
            float dt = currentTimeStep * timeScale;
            UpdateBodiesRK4(dt); // Use RK4 integrator
            simulationTime += dt;
        }

        // --- Drawing ---
        BeginDrawing();
        ClearBackground(DARKGRAY); // Darker background often looks better

        BeginMode2D(camera); // Apply camera transformations
        DrawTrails();
        DrawSimulation();
        EndMode2D();

        DrawUI(); // Draw UI elements outside of camera space

        EndDrawing();
    }

    CloseWindow();
    return 0;
}

// --- Function Implementations ---

void InitializeBodies() {
    // Example: Figure-8 (Requires precise initial conditions and tuning of G/timestep)
    // Note: Getting stable specific orbits like Figure-8 is VERY sensitive.
    // These values might need significant tweaking of G, timestep, softening.
    float scale = 150.0f;
    float v_scale = 7.0f; // Velocity scaling - adjust this significantly!
    bodies[0] = { {-0.97000436f * scale + screenWidth / 2.0f, 0.24308753f * scale + screenHeight / 2.0f}, {0.46620368f * v_scale, 0.43236573f * v_scale}, {0,0}, 30.0f, 10.0f, RED };
    bodies[1] = { {0.0f * scale + screenWidth / 2.0f, 0.0f * scale + screenHeight / 2.0f}, {-0.93240737f * v_scale, -0.86473146f * v_scale}, {0,0}, 10.0f, 20.0f, GREEN };
    bodies[2] = { {0.97000436f * scale + screenWidth / 2.0f, -0.24308753f * scale + screenHeight / 2.0f}, {0.46620368f * v_scale, 0.43236573f * v_scale}, {0,0}, 10.0f, 10.0f, BLUE };

    // Simple Binary with smaller third body
    /*
    bodies[0] = {{screenWidth / 2.0f - 150, screenHeight / 2.0f}, {0.0f, 5.0f}, {0,0}, 100.0f, 15.0f, RED};
    bodies[1] = {{screenWidth / 2.0f + 150, screenHeight / 2.0f}, {0.0f, -5.0f}, {0,0}, 100.0f, 15.0f, GREEN};
    bodies[2] = {{screenWidth / 2.0f, screenHeight / 2.0f + 100 }, {0.0f, 0.0f}, {0,0}, 0.1f, 3.0f, BLUE};
    */

    // Clear trails and add initial point
    for (auto& body : bodies) {
        body.trail.clear();
        AddToTrail(body);
    }
}

void InitializeCamera() {
    camera.target = { screenWidth / 2.0f, screenHeight / 2.0f };
    camera.offset = { screenWidth / 2.0f, screenHeight / 2.0f };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;
}

// Calculates acceleration for all bodies based on a given set of states
void CalculateSystemAccelerations(const std::vector<State>& currentStates, std::vector<Derivative>& outputDerivatives) {
    // Reset accelerations (implicitly done by calculating forces from scratch)
    std::vector<Vector2> forces(bodies.size(), { 0.0f, 0.0f });

    for (int i = 0; i < bodies.size(); ++i) {
        for (int j = i + 1; j < bodies.size(); ++j) {
            Vector2 gravityForce = CalculateGravityFromState(currentStates[i], bodies[i].mass, currentStates[j], bodies[j].mass);
            forces[i] = Vector2Add(forces[i], gravityForce);
            forces[j] = Vector2Subtract(forces[j], gravityForce); // Newton's 3rd Law
        }
    }

    // Calculate derivatives (dpdt = velocity, dvdt = acceleration = F/m)
    for (int i = 0; i < bodies.size(); ++i) {
        outputDerivatives[i].dpdt = currentStates[i].velocity;
        if (bodies[i].mass > 1e-6f) { // Avoid division by zero for massless particles (though unlikely here)
            outputDerivatives[i].dvdt = Vector2Scale(forces[i], 1.0f / bodies[i].mass);
        }
        else {
            outputDerivatives[i].dvdt = { 0.0f, 0.0f };
        }
        // Also store the final calculated acceleration in the body struct for display/use
        bodies[i].acceleration = outputDerivatives[i].dvdt;
    }
}

// Calculate gravitational force between two actual Body objects
Vector2 CalculateGravity(const Body& body1, const Body& body2) {
    Vector2 direction = Vector2Subtract(body2.position, body1.position);
    float distance = Vector2Length(direction);

    if (distance < 1e-6f) return { 0.0f, 0.0f }; // Avoid singularity if bodies overlap exactly

    // Softening: Add to distance squared in the denominator
    float distanceSq = distance * distance;
    float forceMagnitude = G * body1.mass * body2.mass / (distanceSq + softingFactor * softingFactor);

    // Force vector = Magnitude * Normalized Direction
    return Vector2Scale(direction, forceMagnitude / distance);
}

// Calculate gravitational force between two states (needed for RK4 intermediate steps)
Vector2 CalculateGravityFromState(const State& state1, float mass1, const State& state2, float mass2) {
    Vector2 direction = Vector2Subtract(state2.position, state1.position);
    float distance = Vector2Length(direction);

    if (distance < 1e-6f) return { 0.0f, 0.0f };

    float distanceSq = distance * distance;
    float forceMagnitude = G * mass1 * mass2 / (distanceSq + softingFactor * softingFactor);

    return Vector2Scale(direction, forceMagnitude / distance);
}


// Helper function to advance a state by dt * derivative
State StateAddDt(const State& s, const Derivative& d, float dt) {
    State result;
    result.position = Vector2Add(s.position, Vector2Scale(d.dpdt, dt));
    result.velocity = Vector2Add(s.velocity, Vector2Scale(d.dvdt, dt));
    return result;
}


void UpdateBodiesRK4(float dt) {
    int n = bodies.size();
    std::vector<State> initialStates(n);
    std::vector<State> intermediateStates(n);
    std::vector<Derivative> k1(n), k2(n), k3(n), k4(n);

    // 1. Store initial state
    for (int i = 0; i < n; ++i) {
        initialStates[i] = { bodies[i].position, bodies[i].velocity };
    }

    // 2. Calculate k1 (derivatives at initial state)
    CalculateSystemAccelerations(initialStates, k1);

    // 3. Calculate k2 (derivatives at state predicted by k1 after dt/2)
    for (int i = 0; i < n; ++i) intermediateStates[i] = StateAddDt(initialStates[i], k1[i], dt * 0.5f);
    CalculateSystemAccelerations(intermediateStates, k2);

    // 4. Calculate k3 (derivatives at state predicted by k2 after dt/2)
    for (int i = 0; i < n; ++i) intermediateStates[i] = StateAddDt(initialStates[i], k2[i], dt * 0.5f);
    CalculateSystemAccelerations(intermediateStates, k3);

    // 5. Calculate k4 (derivatives at state predicted by k3 after dt)
    for (int i = 0; i < n; ++i) intermediateStates[i] = StateAddDt(initialStates[i], k3[i], dt);
    CalculateSystemAccelerations(intermediateStates, k4);

    // 6. Combine derivatives and update state
    for (int i = 0; i < n; ++i) {
        // Weighted average of derivatives for position (dpdt = velocity)
        Vector2 avg_dpdt = Vector2Scale(
            Vector2Add(k1[i].dpdt,
                Vector2Add(Vector2Scale(k2[i].dpdt, 2.0f),
                    Vector2Add(Vector2Scale(k3[i].dpdt, 2.0f), k4[i].dpdt))),
            1.0f / 6.0f);

        // Weighted average of derivatives for velocity (dvdt = acceleration)
        Vector2 avg_dvdt = Vector2Scale(
            Vector2Add(k1[i].dvdt,
                Vector2Add(Vector2Scale(k2[i].dvdt, 2.0f),
                    Vector2Add(Vector2Scale(k3[i].dvdt, 2.0f), k4[i].dvdt))),
            1.0f / 6.0f);

        // Update position and velocity using the averaged derivatives
        bodies[i].position = Vector2Add(initialStates[i].position, Vector2Scale(avg_dpdt, dt));
        bodies[i].velocity = Vector2Add(initialStates[i].velocity, Vector2Scale(avg_dvdt, dt));

		// not practicle, but reflect the body if it goes out of the screen
		if (bodies[i].position.x < 0 || bodies[i].position.x > screenWidth) {
			bodies[i].velocity.x = -bodies[i].velocity.x;
		}
		if (bodies[i].position.y < 0 || bodies[i].position.y > screenHeight) {
			bodies[i].velocity.y = -bodies[i].velocity.y;
		}

        // Update trail
        AddToTrail(bodies[i]);
    }
}


void AddToTrail(Body& body) {
    body.trail.push_back(body.position);
    if (body.trail.size() > MAX_TRAIL_LENGTH) {
        body.trail.pop_front();
    }
}

void DrawSimulation() {
    for (int i = 0; i < bodies.size(); ++i) {
        // Draw Body
        DrawCircleV(bodies[i].position, bodies[i].radius * camera.zoom, bodies[i].color); // Scale radius slightly with zoom

        // Highlight selected body
        if (i == selectedBody) {
            DrawRing(bodies[i].position,
                (bodies[i].radius + 3.0f) * camera.zoom, // Outer radius
                (bodies[i].radius + 5.0f) * camera.zoom, // Inner radius thicker ring
                0.0f, 360.0f, 0, YELLOW); // Make ring visible
        }
    }
}

void DrawTrails() {
    for (const auto& body : bodies) {
        if (body.trail.size() < 2) continue;

        for (size_t i = 0; i < body.trail.size(); ++i) {
            float alpha = (float)i / body.trail.size(); // Fade out older points
            Color trailColor = ColorAlpha(body.color, alpha * 0.6f); // Adjust alpha multiplier
            // Draw pixel is cheap but small, draw circle looks better but more expensive
            DrawCircleV(body.trail[i], 1.2f, trailColor);
            // For lines (connect the dots style):
            // if (i > 0) DrawLineEx(body.trail[i-1], body.trail[i], 1.5f, trailColor);
        }
    }
}


void DrawUI() {
    // --- General Info ---
    DrawText(TextFormat("Time: %.2f", simulationTime), 10, 10, 20, LIGHTGRAY);
    DrawText(TextFormat("Time Step: %.4f (%.1fx)", currentTimeStep, timeScale), 10, 35, 20, LIGHTGRAY);
    DrawText(paused ? "PAUSED (P)" : "Running (P to Pause)", 10, 60, 20, paused ? YELLOW : LIGHTGRAY);
    DrawText("R: Restart | 1/2/3: Select Body | +/-: Adjust Time Scale", 10, screenHeight - 30, 15, GRAY);
    DrawText("Mouse Wheel: Zoom | MMB Drag: Pan | C: Reset Camera", 10, screenHeight - 50, 15, GRAY);


    // --- Energy Info ---
    double ke = CalculateTotalKineticEnergy();
    double pe = CalculateTotalPotentialEnergy();
    double totalE = ke + pe;
    DrawText(TextFormat("Total KE: %.3e", ke), screenWidth - 250, 10, 18, SKYBLUE);
    DrawText(TextFormat("Total PE: %.3e", pe), screenWidth - 250, 35, 18, LIME);
    DrawText(TextFormat("Total E:  %.3e", totalE), screenWidth - 250, 60, 18, YELLOW);


    // --- Selected Body Info ---
    if (selectedBody >= 0 && selectedBody < bodies.size()) {
        const auto& b = bodies[selectedBody];
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1); // Format floats nicely
        ss << "Selected Body " << (selectedBody + 1) << " [" << ColorToString(b.color) << "]\n";
        ss << " Pos: (" << b.position.x << ", " << b.position.y << ")\n";
        ss << " Vel: (" << b.velocity.x << ", " << b.velocity.y << ") | Mag: " << Vector2Length(b.velocity) << "\n";
        ss << " Acc: (" << b.acceleration.x << ", " << b.acceleration.y << ") | Mag: " << Vector2Length(b.acceleration) << "\n";
        ss << " Mass: " << b.mass << " (Numpad +/-)\n";
        ss << " Radius: " << b.radius << " (Numpad */)";

        DrawText(ss.str().c_str(), 10, 90, 15, WHITE);
        DrawText("Numpad Arrows: Move | Numpad 5/8: Adjust Vel", 10, 180, 15, GRAY);

    }
    else {
        DrawText("No body selected", 10, 90, 15, WHITE);
    }


}

// Helper to get color name (approximated)
const char* ColorToString(Color c) {
    if (c.r == RED.r && c.g == RED.g && c.b == RED.b) return "Red";
    if (c.r == GREEN.r && c.g == GREEN.g && c.b == GREEN.b) return "Green";
    if (c.r == BLUE.r && c.g == BLUE.g && c.b == BLUE.b) return "Blue";
    if (c.r == YELLOW.r && c.g == YELLOW.g && c.b == YELLOW.b) return "Yellow";
    // Add more colors if needed
    return "Custom";
}


double CalculateTotalKineticEnergy() {
    double totalKE = 0.0;
    for (const auto& body : bodies) {
        totalKE += 0.5 * body.mass * Vector2LengthSqr(body.velocity);
    }
    return totalKE;
}

double CalculateTotalPotentialEnergy() {
    double totalPE = 0.0;
    for (int i = 0; i < bodies.size(); ++i) {
        for (int j = i + 1; j < bodies.size(); ++j) {
            Vector2 direction = Vector2Subtract(bodies[j].position, bodies[i].position);
            float distance = Vector2Length(direction);
            if (distance > 1e-6f) { // Avoid division by zero
                // PE = -G * m1 * m2 / r (No softening used in standard PE calc)
                totalPE -= G * bodies[i].mass * bodies[j].mass / distance;
            }
        }
    }
    return totalPE;
}


void UpdateCamera() {
    // Pan (Middle Mouse Button)
    if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
        Vector2 delta = GetMouseDelta();
        delta = Vector2Scale(delta, -1.0f / camera.zoom); // Scale panning by zoom level
        camera.target = Vector2Add(camera.target, delta);
    }

    // Zoom (Mouse Wheel)
    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        // Get the world point that is under the mouse
        Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
        // Set the offset to where the mouse is
        camera.offset = GetMousePosition();
        // Set the target to match the mouse world position
        camera.target = mouseWorldPos;
        // Zoom
        float scaleFactor = 1.0f + (wheel * 0.1f); // Adjust zoom speed
        camera.zoom = Clamp(camera.zoom * scaleFactor, 0.1f, 10.0f); // Clamp zoom level
    }

    // Reset Camera
    if (IsKeyPressed(KEY_C)) {
        InitializeCamera();
    }
}

void HandleInput() {
    if (IsKeyPressed(KEY_R)) {
        RestartSimulation();
    }
    if (IsKeyPressed(KEY_P)) {
        paused = !paused;
    }

    // Select Body
    if (IsKeyPressed(KEY_ONE)) selectedBody = 0;
    if (IsKeyPressed(KEY_TWO)) selectedBody = 1;
    if (IsKeyPressed(KEY_THREE)) selectedBody = 2;

    // Adjust Time Scale
    if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) { // = or + or Numpad +
        timeScale *= 1.5f;
    }
    if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) { // - or Numpad -
        timeScale /= 1.5f;
    }
    timeScale = Clamp(timeScale, 0.1f, 10.0f); // Keep timescale reasonable


    // --- Numpad Controls for Selected Body ---
    if (selectedBody >= 0 && selectedBody < bodies.size()) {
        float posAdjust = 5.0f;
        float velAdjust = 1.0f;
        float massAdjust = bodies[selectedBody].mass * 0.1f + 0.1f; // Adjust mass proportionally
        float radiusAdjust = 1.0f;

        // Position (Numpad Arrows) - Add while key is DOWN
        if (IsKeyDown(KEY_KP_4)) bodies[selectedBody].position.x -= posAdjust; // Left
        if (IsKeyDown(KEY_KP_6)) bodies[selectedBody].position.x += posAdjust; // Right
        if (IsKeyDown(KEY_KP_8)) bodies[selectedBody].position.y -= posAdjust; // Up (screen coords)
        if (IsKeyDown(KEY_KP_2)) bodies[selectedBody].position.y += posAdjust; // Down (screen coords)

        // Velocity (Numpad 5=Decrease, Numpad 0=Increase magnitude? Let's use 8/2 for Y vel)
        // Let's redefine slightly: 8/2 for Y vel, 4/6 for X vel, separate from position
        if (IsKeyDown(KEY_KP_DECIMAL)) bodies[selectedBody].velocity = { 0,0 }; // Numpad '.' to stop

        if (IsKeyPressed(KEY_KP_4)) bodies[selectedBody].velocity.x -= velAdjust;
        if (IsKeyPressed(KEY_KP_6)) bodies[selectedBody].velocity.x += velAdjust;
        if (IsKeyPressed(KEY_KP_8)) bodies[selectedBody].velocity.y -= velAdjust; // Up (negative Y vel)
        if (IsKeyPressed(KEY_KP_2)) bodies[selectedBody].velocity.y += velAdjust; // Down (positive Y vel)


        // Mass (Numpad +/-)
        if (IsKeyDown(KEY_KP_ADD)) bodies[selectedBody].mass += massAdjust;
        if (IsKeyDown(KEY_KP_SUBTRACT)) bodies[selectedBody].mass -= massAdjust;
        bodies[selectedBody].mass = fmaxf(bodies[selectedBody].mass, 0.01f); // Prevent zero/negative mass

        // Radius (Numpad */)
        if (IsKeyDown(KEY_KP_MULTIPLY)) bodies[selectedBody].radius += radiusAdjust;
        if (IsKeyDown(KEY_KP_DIVIDE)) bodies[selectedBody].radius -= radiusAdjust;
        bodies[selectedBody].radius = fmaxf(bodies[selectedBody].radius, 1.0f); // Prevent zero/negative radius

        // If position/velocity was adjusted manually, update initial state IF simulation restarted immediately after
        // This allows tweaking and then restarting from the tweaked state.
        // However, it's usually better to restart to the *original* initial state.
        // initialBodies[selectedBody] = bodies[selectedBody]; // Optional: uncomment to make tweaks persistent after restart
    }

    // Optional: Keep bodies within some bounds (could be removed for free-roam)
    /*
    for (int i = 0; i < bodies.size(); ++i) {
        float safetyMargin = 500.0f; // Allow going off screen
        if (bodies[i].position.x < -safetyMargin) bodies[i].position.x = -safetyMargin;
        if (bodies[i].position.x > screenWidth + safetyMargin) bodies[i].position.x = screenWidth + safetyMargin;
        if (bodies[i].position.y < -safetyMargin) bodies[i].position.y = -safetyMargin;
        if (bodies[i].position.y > screenHeight + safetyMargin) bodies[i].position.y = screenHeight + safetyMargin;
    }
    */
}


void RestartSimulation() {
    for (int i = 0; i < bodies.size(); ++i) {
        bodies[i] = initialBodies[i]; // Reset to original initial state
        // Clear trail and add the reset position
        bodies[i].trail.clear();
        AddToTrail(bodies[i]);
    }
    paused = false;
    simulationTime = 0.0;
    InitializeCamera(); // Reset camera view on restart
    timeScale = 1.0f; // Reset time scale
    std::cout << "Simulation Restarted." << std::endl;
}

// by Wasswa Lutufi Sebbanja