#version 430 core

// Define a uniform struct for lights
struct Light {
    // The matrices are used for shadow mapping. You need to fill it according to how we are filling it when building the normal maps (node_render_shadow_mapping.cpp). 
    // Now, they are filled with identity matrix. You need to modify C++ code innode_render_deferred_lighting.cpp.
    // Position and color are filled.
    mat4 light_projection;
    mat4 light_view;
    vec3 position;
    float radius;
    vec3 color; // Just use the same diffuse and specular color.
    int shadow_map_id;
};

layout(binding = 0) buffer lightsBuffer {
Light lights[4];
};

uniform vec2 iResolution;

uniform sampler2D diffuseColorSampler;
uniform sampler2D normalMapSampler; // You should apply normal mapping in rasterize_impl.fs
uniform sampler2D metallicRoughnessSampler;
uniform sampler2DArray shadow_maps;
uniform sampler2D position;

// uniform float alpha;
uniform vec3 camPos;

uniform int light_count;

layout(location = 0) out vec4 Color;

void main() {
    vec2 uv = gl_FragCoord.xy / iResolution;

    vec3 pos = texture2D(position,uv).xyz;
    vec3 normal = texture2D(normalMapSampler,uv).xyz;

    vec4 metalnessRoughness = texture2D(metallicRoughnessSampler,uv);
    float metal = metalnessRoughness.x;
    float roughness = metalnessRoughness.y;
    vec3 Color_temp = vec3(0.0);

    for(int i = 0; i < light_count; i ++) {

    // ambient color
    vec3 ambient = lights[i].color * 0.2;

    // light depth
    vec4 temp = lights[i].light_projection * lights[i].light_view * (vec4(pos, 1.0));
    vec3 light_uv = ( temp.xyz / temp.w );

    float shadow_map_value = texture(shadow_maps, vec3(0.5*light_uv.xy+0.5, lights[i].shadow_map_id)).x;

    vec3 result;

    if (light_uv.z > shadow_map_value+0.01)
    {
        result = ambient;
    }
    else
    {
        // diffuse color
        vec3 norm = normalize(normal);
        vec3 lightDir = normalize(lights[i].position - pos);
        float diff = max(dot(norm, lightDir), 0.0);
        vec3 diffuse = lights[i].color * (1-metal * 0.8) * diff;

        // specular color
        vec3 viewDir = normalize(camPos - pos);
        vec3 reflectDir = reflect(-lightDir, norm);  
        float spec = pow(max(dot(viewDir, reflectDir), 0.0), (1-roughness)*2);
        vec3 specular = lights[i].color * metal * 0.8 * spec;  

        result = ambient + diffuse + specular;
    }

    // Visualization of shadow map
    result = result * texture2D(diffuseColorSampler,uv).xyz;
    Color_temp = Color_temp + result;
    // HW6_TODO: first comment the line above ("Color +=..."). That's for quick Visualization.
    // You should first do the Blinn Phong shading here. You can use roughness to modify alpha. Or you can pass in an alpha value through the uniform above.

    // After finishing Blinn Phong shading, you can do shadow mapping with the help of the provided shadow_map_value. You will need to refer to the node, node_render_shadow_mapping.cpp, for the light matrices definition. Then you need to fill the mat4 light_projection; mat4 light_view; with similar approach that we fill position and color.
    // For shadow mapping, as is discussed in the course, you should compare the value "position depth from the light's view" against the "blocking object's depth.", then you can decide whether it's shadowed.

    // PCSS is also applied here.
    }
    Color = vec4(Color_temp, 1.0);
}