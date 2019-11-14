#version 330
uniform int color_mode;
uniform vec3 intensity;
uniform vec3 light_color;

in vec3 fcolor;
in vec3 fnormal;
in vec3 view_dir;
in vec3 light_dir;

out vec4 color;

void main() {
    vec3 c = vec3(0.0);
    if (color_mode == 0) {
        c += vec3(1.0)*vec3(0.1, 0.1, 0.1);
        vec3 n = normalize(fnormal);
        vec3 v = normalize(view_dir);
        vec3 l = normalize(light_dir);
        float lambert = dot(n,l);
        if(lambert > 0.0) {
            c += vec3(1.0)*light_color*lambert;
            vec3 v = normalize(view_dir);
            vec3 r = reflect(-l,n);
            c += vec3(1.0)*vec3(0.8, 0.8, 0.8)*pow(max(dot(r,v), 0.0), 90.0);
        }
        c *= fcolor;
    } else {
       c = fcolor;
    }
    if (intensity == vec3(0.0)) {
        c = intensity;
    }
    color = vec4(c, 1.0);
}
