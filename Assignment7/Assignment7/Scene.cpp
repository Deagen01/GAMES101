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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    Vector3f eye_pos = ray.origin;//这是p
    Vector3f wo = ray.direction;//像素指向物体
    //查找该光线与场景的交点
    Intersection hitPoint = intersect(ray);
    if(!hitPoint.happened && depth==0){
        return this->backgroundColor;
    }
    if(!hitPoint.happened){
        return Vector3f(0.f);
    }
    //判断打到的物体是否发光
    Vector3f L_e = {0.f,0.f,0.f};
    //物体发光 是L_e 自发光项
    if(hitPoint.m->hasEmission()){
        L_e = hitPoint.m->getEmission();
        return L_e;//只需要返回自发光项？
    }
    //直接光照
    Vector3f L_dir = {0.f,0.f,0.f};
    Intersection light_pos;//采样的光源交点
    float pdf;
    //对光源进行随机采样 采样一个wi出来 
    sampleLight(light_pos, pdf);
    //构造从该光源出发的光线
    Ray lightRay = Ray(light_pos.coords, normalize(hitPoint.coords - light_pos.coords));
    float dis = (light_pos.coords - hitPoint.coords).norm();
    //获取该光线与场景的交点
    Intersection lightInter = intersect(lightRay);
    if(lightInter.happened && (dis - lightInter.distance) < EPSILON){
        //计算直接光照
        Vector3f xx = light_pos.coords;
        Vector3f ws=normalize(hitPoint.coords - xx);//光源指向hitPoint
        Vector3f NN=light_pos.normal;
        Vector3f L_i = light_pos.emit;
        float dis2 = dis*dis;
        Vector3f f_r = hitPoint.m->eval(wo,-ws, hitPoint.normal);
        L_dir = L_i*f_r*dotProduct(-ws, hitPoint.normal)*dotProduct(ws, NN)/dis2/pdf;
    }
    //计算间接光照
    //随机生成一个半球的光照方向 
    Vector3f wi = normalize(hitPoint.m->sample(wo,hitPoint.normal));
    Ray rayWi = Ray(hitPoint.coords, wi);
    Intersection inter_wi = intersect(rayWi);
    //若打到发光物体则则属于直接光照 就不处理
    //只有打到未发光物体才属于间接光照要处理
    Vector3f L_indir = {0.f,0.f,0.f};
    if(!inter_wi.happened){//一定可以打到物体才对
        return L_dir;
    }
    if(!inter_wi.m->hasEmission()){
        //使用俄罗斯轮盘来判断是否追踪光线
        float p_rr = get_random_float();
        if(p_rr<RussianRoulette){
            Vector3f L_i=castRay(rayWi, depth+1);
            Vector3f f_r = hitPoint.m->eval(wo,wi,hitPoint.normal);
            //这里的概率密度是谁的概率密度？是hitPoint中omega的概率密度
            float pdf_wi = inter_wi.m->pdf(wo,wi,hitPoint.normal);
            float dis = (inter_wi.coords-hitPoint.coords).norm();
            float dis2 = dis*dis;
            //这里除以0了
            L_indir = L_i*f_r*dotProduct(wi,hitPoint.normal)/pdf_wi/RussianRoulette;
            if(L_indir.norm()<EPSILON)
            {
                L_indir.x=0.f;
            }
        }

        
    }

    return L_dir+L_indir;

}