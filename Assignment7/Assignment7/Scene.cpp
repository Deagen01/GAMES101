//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    //计算场景中的总光源面积
    for (uint32_t k = 0; k < objects.size(); ++k) {
        //遍历场景中所有物体
        //寻找发光物体
        //计算发光物体的总面积
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    //随机采样一个光源
    //先生成一个随机数，然后判断它是位于哪一个物体光源上
    //再对该物体进行光源采样
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            //判断这个随机位置是否是在当前第K个光源上
            //若在则直接采样
            //不在继续遍历光源
            if (p <= emit_area_sum){
                //记录本次采样的光源位置和pdf
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
// Vector3f Scene::castRay(const Ray &ray, int depth) const
// {
//     // TO DO Implement Path Tracing Algorithm here

//     Vector3f eye_pos = ray.origin;//这是p
//     Vector3f wo = ray.direction;//像素指向物体
//     //查找该光线与场景的交点
//     Intersection hitPoint = intersect(ray);
//     if(!hitPoint.happened && depth==0){
//         return this->backgroundColor;
//     }
//     if(!hitPoint.happened){
//         return Vector3f(0.f);
//     }
//     //判断打到的物体是否发光
//     Vector3f L_e = {0.f,0.f,0.f};
//     //物体发光 是L_e 自发光项
//     if(hitPoint.m->hasEmission()){
//         L_e = hitPoint.m->getEmission();
//         return L_e;//只需要返回自发光项？
//     }
//     //直接光照
//     Vector3f L_dir = {0.f,0.f,0.f};
//     Intersection light_pos;//采样的光源交点
//     float pdf;
//     //对光源进行随机采样 采样一个wi出来 
//     sampleLight(light_pos, pdf);
//     //构造从该光源出发的光线
//     Ray lightRay = Ray(light_pos.coords, normalize(hitPoint.coords - light_pos.coords));
//     float dis = (light_pos.coords - hitPoint.coords).norm();
//     //获取该光线与场景的交点
//     Intersection lightInter = intersect(lightRay);
//     if(lightInter.happened && (dis - lightInter.distance) < EPSILON){
//         //计算直接光照
//         Vector3f xx = light_pos.coords;
//         Vector3f ws=normalize(hitPoint.coords - xx);//光源指向hitPoint
//         Vector3f NN=light_pos.normal;
//         Vector3f L_i = light_pos.emit;
//         float dis2 = dis*dis;
//         Vector3f f_r = hitPoint.m->eval(wo,-ws, hitPoint.normal);
//         L_dir = L_i*f_r*dotProduct(-ws, hitPoint.normal)*dotProduct(ws, NN)/dis2/pdf;
//     }
//     //计算间接光照
//     //随机生成一个半球的光照方向 
//     Vector3f wi = normalize(hitPoint.m->sample(wo,hitPoint.normal));
//     Ray rayWi = Ray(hitPoint.coords, wi);
//     Intersection inter_wi = intersect(rayWi);
//     //若打到发光物体则则属于直接光照 就不处理
//     //只有打到未发光物体才属于间接光照要处理
//     Vector3f L_indir = {0.f,0.f,0.f};
//     if(!inter_wi.happened){//一定可以打到物体才对
//         return L_dir;
//     }
//     if(!inter_wi.m->hasEmission()){
//         //使用俄罗斯轮盘来判断是否追踪光线
//         float p_rr = get_random_float();
//         if(p_rr<RussianRoulette){
//             Vector3f L_i=castRay(rayWi, depth+1);
//             Vector3f f_r = hitPoint.m->eval(wo,wi,hitPoint.normal);
//             //这里的概率密度是谁的概率密度？是hitPoint中omega的概率密度
//             float pdf_wi = inter_wi.m->pdf(wo,wi,hitPoint.normal);
//             float dis = (inter_wi.coords-hitPoint.coords).norm();
//             float dis2 = dis*dis;
//             //这里除以0了
//             L_indir = L_i*f_r*dotProduct(wi,hitPoint.normal)/pdf_wi/RussianRoulette;
//             if(L_indir.norm()<EPSILON)
//             {
//                 L_indir.x=0.f;
//             }
//         }

        
//     }

//     return L_dir+L_indir;

// }

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    //创建变量以储存直接和间接光照计算值
    Vector3f dir = { 0.0,0.0,0.0 };
    Vector3f indir = { 0.0,0.0,0.0 };
    //1.判断是否有交点：光线与场景中物体相交？
    Intersection inter = Scene::intersect(ray);
    //如果没交点
    if (!inter.happened) {
        return dir;//return 0,0,0
    }
    //2.ray打到光源了：说明渲染方程只用算前面的自发光项，因此直接返回材质的自发光项
    if (inter.m->hasEmission()) {
        if (depth == 0) {//第一次打到光
            return inter.m->getEmission();
        }
        else return dir;//弹射打到光，直接返回0，0.0
 
    }
    //3.ray打到物体：这个时候才开始进行伪代码后面的步骤
    
    //对场景中的光源进行采样，得到采样点light_pos和pdf_light
    Intersection light_pos;
    float pdf_light = 0.0f;
    sampleLight(light_pos, pdf_light);
   
    //3.1计算直接光照
 
    //物体的一些参数
    Vector3f p = inter.coords;
    Vector3f N = inter.normal.normalized();
    Vector3f wo = ray.direction;//物体指向场景
    //光源的一些参数
    Vector3f xx = light_pos.coords;
    Vector3f NN = light_pos.normal.normalized();
    Vector3f ws = (p - xx).normalized();//光源指向物体
    float dis = (p - xx).norm();//二者距离
    float dis2 = dotProduct((p - xx), (p - xx));
 
    //判断光源与物体间是否有遮挡：
    //发出一条射线，方向为ws 光源xx -> 物体p
    Ray light_to_obj(xx, ws);//Ray(orig,dir)
    Intersection light_to_scene = Scene::intersect(light_to_obj);
    //假如dis>light_to_scene.distance就说明有遮挡，那么反着给条件即可：
    // if (light_to_scene.happened&& (light_to_scene.distance-dis>-EPSILON)) {//没有遮挡
    //     //为了更贴近伪代码，先设定一些参数
    //     Vector3f L_i = light_pos.emit;//光强
    //     Vector3f f_r = inter.m->eval(wo, -ws, N);//材质，课上说了，BRDF==材质，ws不参与计算
    //     float cos_theta = dotProduct(-ws, N);//物体夹角
    //     float cos_theta_l = dotProduct(ws, NN);//光源夹角
    //     dir = L_i * f_r * cos_theta * cos_theta_l / dis2 / pdf_light;
    // }
 
    //3.2间接光照
    
    //俄罗斯轮盘赌
    //Scene.hpp中已经定义了P_RR:RussianRoulette=0.8
    float ksi = get_random_float();//随机取[0,1]
    if (ksi < RussianRoulette) {
        //计算间接光照
        
        //随机生成一个wi方向
        Vector3f wi = inter.m->sample(wo, N).normalized();//这里的wi其实没参与计算，返回的是一个随机的方向
        Ray r(p, wi);
        Intersection obj_to_scene = Scene::intersect(r);
        //击中了物体&&物体不是光源
        if (obj_to_scene.happened && !obj_to_scene.m->hasEmission()) {
            Vector3f f_r = inter.m->eval(wo, wi, N);//wo不参与计算
            float cos_theta = dotProduct(wi, N);
            float pdf_hemi = inter.m->pdf(wo, wi, N);
            indir = castRay(r, depth + 1) * f_r * cos_theta / pdf_hemi / RussianRoulette;
        }
    }
    return dir + indir;
}