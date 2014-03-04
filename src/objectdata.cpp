#include "objectdata.h"
ObjectData* ObjectData::_od = NULL;

ObjectData::ObjectData()
{
    stream = new std::ifstream("objects.xml");
    doc = new xml_document();
    doc->load(*(stream));
    
    for (xml_node obj = doc->child("objects").child("object"); obj; obj = obj.next_sibling("object"))
    {             
        ImageObject object;
        object.name = obj.child("name").child_value();
        object.h_min = toInt(obj.child("hmin").child_value());
        object.h_max = toInt(obj.child("hmax").child_value());
        object.s_min = toInt(obj.child("smin").child_value());
        object.s_max = toInt(obj.child("smax").child_value());
        object.v_min = toInt(obj.child("vmin").child_value());
        object.v_max = toInt(obj.child("vmax").child_value());
        objs.push_back(object);
    }
}

ObjectData& ObjectData::getInstance() {
    if(ObjectData::_od == NULL)
        ObjectData::_od = new ObjectData();
    return *ObjectData::_od;
}

void ObjectData::showObjectsName() {
    for(int i = 0; i<objs.size(); i++)
        std::cout << std::endl << "(" << objs.at(i).name << ")" << std::endl;
}

ImageObject *ObjectData::get(int i) {
    if(i >= 0 && i < objs.size())
        return &objs.at(i);
    else
        return 0;
}

ImageObject *ObjectData::get(std::string str) {
    for(int i = 0; i<objs.size(); i++)
        if(str == objs.at(i).name)
            return &objs.at(i);
    return 0;
}

void ObjectData::save(std::string name, int hmin, int hmax, int smin, int smax, int vmin, int vmax)
{
    bool exist = false;
    for(int i = 0; i<objs.size(); i++)
        if(name == objs.at(i).name)
            exist = true;
    
    if(exist) {
        xml_node obj;
        for(obj = doc->child("objects").child("object"); obj; obj = obj.next_sibling())
            if(obj.child("name").child_value() == name)
                break;
        
        obj.child("hmin").text().set(hmin);
        obj.child("hmax").text().set(hmax);
        obj.child("smin").text().set(smin);
        obj.child("smax").text().set(smax);
        obj.child("vmin").text().set(vmin);
        obj.child("vmax").text().set(vmax);
    }
    else {
        xml_node obj = doc->child("objects").append_child("object");
        obj.append_child("name").text().set(name.c_str());
        obj.append_child("hmin").text().set(hmin);
        obj.append_child("hmax").text().set(hmax);
        obj.append_child("smin").text().set(smin);
        obj.append_child("smax").text().set(smax);
        obj.append_child("vmin").text().set(vmin);
        obj.append_child("vmax").text().set(vmax);
    }
    
    doc->save_file("objects.xml");
}

void ObjectData::test()
{
    doc->child("objects").child("object").next_sibling("object").child("hmin").text().set(1992);
    doc->save_file("objects.xml");
}

int ObjectData::toInt(std::string str) {
    std::istringstream buffer(str);
    int v;
    buffer >> v;
    return v;
}
