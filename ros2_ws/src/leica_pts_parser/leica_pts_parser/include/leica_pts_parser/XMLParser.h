#ifndef LEICA_XML_PARSER_H
#define LEICA_XML_PARSER_H

#include <vector>
#include <map>
#include <tinyxml2.h>
#include <leica_pts_parser/SDBParser.h>

class LeicaXMLParser {
    public:
        struct Offset {
            double easting;
            double northing;
            double elevation;
            Offset() : easting(0), northing(0), elevation(0) {}
            Offset(double e, double n, double h) : easting(e), northing(n), elevation(h) {}
        };

        typedef std::map<std::string,Offset> OffsetMap;
    protected:
        tinyxml2::XMLDocument doc;
        std::vector<LeicaSDBParser> parsers;
        std::vector<std::vector<LeicaScanPoint>> scans; 
        std::vector<std::vector<LeicaScanPoint>> centered_scans; 
        std::vector<std::string> names;
        std::vector<std::string> frame_names;
        std::vector<Offset> frame_origins;

        std::string dirname(const std::string & filename) {
            std::string dir = filename;
            size_t pos = filename.find_last_of("/");
            if (pos == std::string::npos) {
                return ".";
            }
            return filename.substr(0,pos);
        }

        OffsetMap offsets,surveyedPoints;



    public:
        LeicaXMLParser() {}

        bool load(const std::string & filename, bool swap_en=false) {
            offsets.clear();
            names.clear();
            frame_names.clear();
            frame_origins.clear();
            scans.clear();
            if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
                return false;
            }
            tinyxml2::XMLElement * ref = NULL;
            tinyxml2::XMLElement * elem;
            
            elem = doc.FirstChildElement("LandXML")->FirstChildElement("HexagonLandXML")->FirstChildElement("Point");
            while (elem) {
                std::string uniqueID=elem->Attribute("uniqueID");
                std::string classType=elem->Attribute("class");
                if (classType == "reference") {
                    ref = elem->FirstChildElement("Coordinates")->FirstChildElement("Local")->FirstChildElement("Grid");
                    Offset o(ref->FloatAttribute("e"),ref->FloatAttribute("n"),ref->FloatAttribute("hghthO"));
                    offsets[uniqueID]=o;
                    surveyedPoints[uniqueID]=o;
                    printf("Stored point '%s' (%f,%f,%f)\n",uniqueID.c_str(),o.easting,o.northing,o.elevation);
                } else if (classType == "measured") {
                    tinyxml2::XMLElement *p = elem->FirstChildElement("Coordinates")->FirstChildElement("Local")->FirstChildElement("Grid");
                    Offset o(p->FloatAttribute("e"),p->FloatAttribute("n"),p->FloatAttribute("hghthO"));
                    surveyedPoints[uniqueID]=o;
                }
                elem = elem->NextSiblingElement("Point");
            }
            if (!ref) {
                printf("Could not find reference point\n");
                return false;
            }

            std::map<std::string,std::string> refStamped;
            elem = doc.FirstChildElement("LandXML")->FirstChildElement("CgPoints")->FirstChildElement("CgPoint");
            while (elem) {
                std::string uniqueID = elem->Attribute("name");
                std::string timeStamp = elem->Attribute("timeStamp");
                if (offsets.find(uniqueID)!=offsets.end()) {
                    refStamped[timeStamp] = uniqueID;
                    printf("Time stamp '%s': %s\n",uniqueID.c_str(),timeStamp.c_str());
                }
                elem = elem->NextSiblingElement("CgPoint");
            }

            tinyxml2::XMLElement *survey = doc.FirstChildElement("LandXML")->FirstChildElement("HexagonLandXML")->FirstChildElement("Survey")->FirstChildElement("InstrumentSetup");
            while (survey) {
                const char * uniqueID=survey->Attribute("uniqueID");
                printf("Survey: %s\n",uniqueID);
                tinyxml2::XMLElement *scan = survey->FirstChildElement("Scan");
                while (scan) {
                    int numImages = scan->IntAttribute("PanoramaImage");
                    std::string scanStamp = scan->Attribute("TimeStarted");
                    elem = scan->FirstChildElement("landxml:DocFileRef");
                    std::string name=elem->Attribute("name");
                    std::string dir=elem->Attribute("location");
                    tinyxml2::XMLElement *scandef = scan->FirstChildElement("ScanDefinition")->FirstChildElement();
                    std::string scanType(scandef->Name());
                    size_t byte_offset = 520;
                    if (numImages == 0) {
                        byte_offset = 504;
                    } else {
                        byte_offset = 520;
                    }
                    printf("Scan: file %s dir %s offset %d images %d stamp %s\n", name.c_str(), dir.c_str(),int(byte_offset),numImages, scanStamp.c_str());
                    LeicaSDBParser ldp;
                    std::string sdbfile = dirname(filename)+"/"+dir+"/"+name+".sdb";
                    if (ldp.load(sdbfile,byte_offset,numImages>0)) {
                        printf("Loaded SDB file '%s': %d points\n",sdbfile.c_str(),int(ldp.getPoints().size()));
                        std::map<std::string,std::string>::const_iterator tit = refStamped.lower_bound(scanStamp); 
                        if (tit == refStamped.begin()) {
                            printf("Warning: no reference point created before scan %s\n",scanStamp.c_str());
                            scans.push_back(ldp.getPoints());
                            centered_scans.push_back(ldp.getPoints());
                            frame_names.push_back("");
                            frame_origins.push_back(Offset());
                            parsers.push_back(ldp);
                        } else {
                            tit --;
                            OffsetMap::const_iterator it = offsets.find(tit->second);
                            if (it != offsets.end()) {
                                frame_names.push_back(it->first);
                                Offset o = it->second;
                                if (swap_en) {
                                    std::swap(o.easting,o.northing);
                                }
                                frame_origins.push_back(o);
                                printf("Adding offset '%s': %f %f %f (%s)\n",it->first.c_str(),
                                        it->second.easting,it->second.northing,it->second.elevation,tit->first.c_str());
                                size_t i = scans.size();
                                centered_scans.push_back(ldp.getPoints());
                                scans.push_back(ldp.getPoints());
                                for (size_t j=0;j<scans[i].size();j++) {
                                    LeicaScanPoint & po = scans[i][j];
                                    if (swap_en) {
                                        po.y += it->second.easting;
                                        po.x += it->second.northing;
                                    } else {
                                        po.x += it->second.easting;
                                        po.y += it->second.northing;
                                    }
                                    po.z += it->second.elevation;
                                }
                                ldp.setPoints(scans[i]);
                                parsers.push_back(ldp);
                            } else {
                                scans.push_back(ldp.getPoints());
                                centered_scans.push_back(ldp.getPoints());
                                frame_names.push_back("");
                                frame_origins.push_back(Offset());
                                parsers.push_back(ldp);
                            }
                        }
                        names.push_back(name);
                    } else {
                        printf("Could not load '%s'. Skipping\n",sdbfile.c_str());
                    }
                    scan = scan->NextSiblingElement("Scan");
                }
                survey = survey->NextSiblingElement("InstrumentSetup");
            }

            return true;
        }

        size_t numScans() const {
            return scans.size();
        }

        const std::vector<LeicaScanPoint> & getPoints(size_t i=0) const {
            return scans[i];
        }

        const std::vector<LeicaScanPoint> & getCenteredPoints(size_t i=0) const {
            return centered_scans[i];
        }

        const std::string & getScanName(size_t i=0) const {
            return names[i];
        }

        const std::string & getFrameName(size_t i=0) const {
            return frame_names[i];
        }

        bool exportMesh(size_t i, double edge_threshold,double vtk_decimate,
            mesh_msgs::msg::MeshGeometry * m_mesh, 
            mesh_msgs::msg::MeshVertexColors * m_color, 
            shape_msgs::msg::Mesh * s_mesh,bool append,  
            const std::string & filename, bool save_vtk, bool save_ply, bool reconstruct_volume) const {
            if (i>=parsers.size()) {
                return  false;
            }
            const Offset & o = frame_origins[i];
            parsers[i].exportMesh(edge_threshold,vtk_decimate,
                    m_mesh,m_color,s_mesh,append,filename,save_vtk,save_ply,reconstruct_volume,
                    o.easting,o.northing,o.elevation);
            return true;
        }

        const OffsetMap & getSurveyedPoints() const {
            return surveyedPoints;
        }
};


#endif // LEICA_XML_PARSER_H
