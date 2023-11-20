PluginInfo@ GetPluginInfo() {
    auto info = PluginInfo();
    info.Name = "Speedslide Script";
    info.Author = "Tomashu";
    info.Version = "v1.0.0";
    info.Description = "Shows speedslide quality as a graph";
    return info;
}

class CFuncKeysReal {
    //CFuncKeysReal() {}
    CFuncKeysReal(const CFuncKeysReal &in other) {
        this.xs = other.xs;
        this.ys = other.ys;
    }
    CFuncKeysReal(array<float> xs, array<float> ys) {
        this.xs = xs;
        this.ys = ys;
    }

    array<float> xs;
    array<float> ys;
    
    void GetBoundingIndices(float x, int& l, int& r) const {
        uint size = xs.Length;
        if (size == 0 or size != ys.Length) {
            l = -1;
            r = -1;
            return;
        }

        if (size == 1 or x < xs[0] - 1e-05) {
            l = 0;
            r = 0;
            return;
        }

        int last = size - 1;
        if (x > xs[last] + 1e-05) {
            l = last;
            r = last;
            return;
        }
        
        r = l;
        for (uint i = 0; i < size; ++i) {
            l = r++;
            if (r > last) {
                r = 0;
            }
            if (x >= (xs[l] - 1e-05) and x <= (xs[r] + 1e-05)) {
                return;
            }
        }
    }
    
    bool ComputeBlendCoef(float x, int& l, int& r, float& blend) const {
        GetBoundingIndices(x, l, r);
        blend = 0;
        if (l == -1) {
            return false;
        }
        if (l == r) {
            return true;
        }

        float lx = xs[l];
        float rx = xs[r];
        float diff = rx - lx;

        if (Math::Abs(diff) < 1e-05) {
            return true;
        }

        blend = ((x - lx) / diff);
        return true;
    }

    float GetValue(float x, bool linear = true) const {
        int l = 0;
        int r = 0;
        float blend = 0.0;
        if (ComputeBlendCoef(x, l, r, blend)) {
            if (linear) {
                return ys[r] * blend + (1 - blend) * ys[l];
            } else {
                return ys[l];
            }
        }
        return 0;
    }
    
    void Scale(float sx, float sy) {
        for (uint i = 0; i < xs.Length; ++i) {
            xs[i] /= sx;
        }
        for (uint i = 0; i < ys.Length; ++i) {
            ys[i] *= sy;
        }
    }
    CFuncKeysReal Scaled(float sx, float sy) const {
        CFuncKeysReal that = this;
        that.Scale(sx, sy);
        return that;
    }
    void Inverse() {
        auto tmp = xs;
        xs = ys;
        ys = tmp;
    }
    CFuncKeysReal Inversed() const {
        CFuncKeysReal that = this;
        that.Inverse();
        return that;
    }
}

const CFuncKeysReal MaxSideFrictionFromSpeedForStadiumCar({0, 100, 200, 300, 400, 500}, {80, 80, 75, 67, 60, 55});

// material_max_side_friction_multiplier
// for platform and road: 1.0
// for dirt:              0.25
// for grass:             0.15
// for air:               0.0

float GetPerfectSpeedslideAbsoluteLocalSpeedXForStadiumCar(float speed_z, float material_max_side_friction_multiplier = 1.0) {
    return (MaxSideFrictionFromSpeedForStadiumCar.GetValue(speed_z * 3.6) * material_max_side_friction_multiplier) / 10;
}

// minimum speed_x to start gaining speed
float GetMinimumSpeedslideAbsoluteLocalSpeedXForStadiumCar(float speed_z, float material_max_side_friction_multiplier = 1.0) {
    return GetPerfectSpeedslideAbsoluteLocalSpeedXForStadiumCar(speed_z, material_max_side_friction_multiplier) / 2;
}

// the closer to 1.0, the better
float GetSpeedslideQualityForStadiumCar(float speed_x, float speed_z, float material_max_side_friction_multiplier = 1.0) {
    float maxsidefriction = MaxSideFrictionFromSpeedForStadiumCar.GetValue(speed_z * 3.6) * material_max_side_friction_multiplier;
    float sidefriction = 20 * Math::Abs(speed_x);
    if (sidefriction > maxsidefriction) {
        return (sidefriction - maxsidefriction) / maxsidefriction;
    }
    return 0.0;
}

vec3 Mult(const vec3& v, const mat3& m) {
    return vec3(
        v.x * m.x.x + v.y * m.x.y + v.z * m.x.z,
        v.x * m.y.x + v.y * m.y.y + v.z * m.y.z,
        v.x * m.z.x + v.y * m.z.y + v.z * m.z.z
    );
}
vec3 MultTranspose(const vec3& v, const mat3& m) {
    return vec3(
        v.x * m.x.x + v.y * m.y.x + v.z * m.z.x,
        v.x * m.x.y + v.y * m.y.y + v.z * m.z.y,
        v.x * m.x.z + v.y * m.y.z + v.z * m.z.z
    );
}

bool valid = false;
float quality = 0.0;
bool left = false;

void OnRunStep(SimulationManager@ simManager) {
    float material_max_side_friction_multiplier = 0.0;
    uint wheels = 0;
    for (uint i = 0; i < 4; ++i) {
        auto@ rt = simManager.Wheels[i].RTState;
        if (rt.HasGroundContact) {
            ++wheels;
            TM::PlugSurfaceMaterialId matid = TM::PlugSurfaceMaterialId(uint8(rt.ContactMaterialId));
            if (matid == TM::PlugSurfaceMaterialId::Dirt) {
                material_max_side_friction_multiplier += 0.25;
            }
            else if (matid == TM::PlugSurfaceMaterialId::Grass) {
                material_max_side_friction_multiplier += 0.15;
            }
            else {
                material_max_side_friction_multiplier += 1;
            }
        }
    }
    if (wheels == 0) {
        valid = false;
        return;
    }
    material_max_side_friction_multiplier /= wheels; // make average

    //log("" + material_max_side_friction_multiplier);

    vec3 local_speed = MultTranspose(simManager.Dyna.CurrentState.LinearSpeed, simManager.Dyna.CurrentState.Location.Rotation);
    quality = GetSpeedslideQualityForStadiumCar(local_speed.x, local_speed.z, material_max_side_friction_multiplier);
    valid = quality > 0;
    left = local_speed.x < 0;
    if (valid && GetVariableBool("sd_quality_print")) {
        log("" + simManager.TickTime + " | " + quality);
    }
}

bool enabled;

void Render() {
    if (GetVariableBool("sd_quality_slider") && UI::Begin("SD Quality")) {
        if (valid) {
            if (left) {
                UI::SliderFloat("", -quality, -2, 0);
            } else {
                UI::SliderFloat("", quality, 0, 2);
            }
        } else {
            UI::BeginDisabled();
            UI::SliderFloat("", 0, -2, 2);
            UI::EndDisabled();
        }
        UI::End();
    }
}

void Main() {
    RegisterVariable("sd_quality_slider", true);
    RegisterVariable("sd_quality_print", false);
}
