#ifndef LEICA_SDB_PARSER_H
#define LEICA_SDB_PARSER_H

#define HAS_VTK
#ifdef HAS_VTK
#include <vtkCellArray.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataWriter.h>
#include <vtkPLYWriter.h>
#include <vtkTriangleFilter.h>
#include <vtkDecimatePro.h>
#include <vtkTriangleStrip.h>
#include <vtkIdFilter.h>
#include <vtkFeatureEdges.h>
#include <vtkTriangle.h>
// #include <vtkCellArrayIterator.h>
#endif

#define HAS_MESH_MSGS
#ifdef HAS_MESH_MSGS
#include <mesh_msgs/msg/mesh_geometry.hpp>
#include <mesh_msgs/msg/mesh_vertex_colors.hpp>
#endif
#include <shape_msgs/msg/mesh.hpp>

struct LeicaScanPoint {
    size_t id;
    float x,y,z;
    unsigned short intensity,snr;
    unsigned short hz,vt;
    unsigned char r,g,b;

    LeicaScanPoint() {}
    
    static float floatFromBuffer(const unsigned char * buffer) {
        float f;
        char *b=(char*)&f;
        b[0]=buffer[0];
        b[1]=buffer[1];
        b[2]=buffer[2];
        b[3]=buffer[3];
        return f;
    }

    static float shortFromBuffer(const unsigned char * buffer) {
        unsigned short s = buffer[1];
        s = (s << 8) | buffer[0];
        return s;
    }

    LeicaScanPoint(const unsigned char * buffer, bool has_rgb) {
        x = floatFromBuffer(buffer+0);
        y = floatFromBuffer(buffer+4);
        z = floatFromBuffer(buffer+8);
        intensity = shortFromBuffer(buffer+12);
        snr = shortFromBuffer(buffer+14);
        hz = shortFromBuffer(buffer+16);
        vt = shortFromBuffer(buffer+18);
        if (has_rgb) {
            r = buffer[20];
            g = buffer[21];
            b = buffer[22];
        } else {
            r = g = b = 0x80;
        }
    }
};


class LeicaSDBParser {
    public:
        typedef std::map<unsigned short, LeicaScanPoint> ScanColumn;
        typedef std::map<unsigned short, ScanColumn> ScanMap;
    protected:
        std::vector<LeicaScanPoint> points;
        ScanMap scan;
        unsigned short min_hz,max_hz;
        unsigned short min_vt,max_vt;


    public:
        LeicaSDBParser() {}

        bool load(const std::string & filename, const size_t offset=520, bool has_rgb=true) {
            points.clear();
            FILE *fp=fopen(filename.c_str(),"r");
            if (!fp) {
                return false;
            }
            char header[offset];
            if (fread(header,1,offset,fp)!=offset) {
                fclose(fp);
                return false;
            }
            while (!feof(fp)) {
                const unsigned int blocksize=has_rgb?24:20;
                unsigned char block[blocksize];
                if (fread(block,1,blocksize,fp)!=blocksize) {
                    break;
                }
                LeicaScanPoint L(block,has_rgb);
                L.id = points.size();
                if (L.id==0) {
                    min_hz = L.hz;
                    max_hz = L.hz;
                    min_vt = L.vt;
                    max_vt = L.vt;
                } else {
                    min_hz = std::min(min_hz,L.hz);
                    max_hz = std::max(max_hz,L.hz);
                    min_vt = std::min(min_vt,L.vt);
                    max_vt = std::max(max_vt,L.vt);
                }
                points.push_back(L);
                scan[L.hz][L.vt] = L;
            }

            fclose(fp);
            return true;
        }

        bool hasPoint(unsigned short hz, unsigned short vt, LeicaScanPoint & p) const {
            ScanMap::const_iterator it = scan.find(hz);
            if (it == scan.end()) return false;
            ScanColumn::const_iterator jt = it->second.find(vt);
            if (jt == it->second.end()) return false;
            p = jt->second;
            return true;
        }


        const std::vector<LeicaScanPoint> & getPoints() const {
            return points;
        }

        // Replace after update
        void setPoints(const std::vector<LeicaScanPoint> & pts) {
            points = pts;
        }

        const ScanMap & getStructuredScan() const {
            return scan;
        }

        static double sqr(double x) {
            return x*x;
        }
        static double dist(const LeicaScanPoint & p, const LeicaScanPoint & q) {
            return sqrt(sqr(p.x-q.x)+sqr(p.y-q.y)+sqr(p.z-q.z));
        }

        bool exportMesh(double edge_threshold,double vtk_decimate,
#ifdef HAS_MESH_MSGS
            mesh_msgs::msg::MeshGeometry * m_mesh, mesh_msgs::msg::MeshVertexColors * m_color, 
#endif
            shape_msgs::msg::Mesh * s_mesh, bool append, 
            const std::string & filename, bool save_vtk, bool save_ply, 
            bool reconstruct_volume, 
            double volume_origin_x=0, double volume_origin_y=0,double volume_origin_z=0) const {
#ifndef HAS_VTK
            return false;
#else
            unsigned int origin_id = 0;
            vtkUnsignedCharArray* colors = vtkUnsignedCharArray::New();
            colors->SetNumberOfComponents(3);
            colors->SetName("colour");
            //vtkFloatArray* intensities = vtkFloatArray::New();
            //intensities->SetNumberOfComponents(1);
            //intensities->SetName("Intensity");
            //vtkFloatArray* snrs = vtkFloatArray::New();
            //snrs->SetNumberOfComponents(1);
            //snrs->SetName("SNR");
            vtkPoints* vpoints = vtkPoints::New();
            // FILE *fp=fopen("out","w");
            for (size_t i=0;i<points.size();i++) {
                // fprintf(fp,"%f %f %f %d %d\n",points[i].x, points[i].y, points[i].z, points[i].hz,points[i].vt);
                vpoints->InsertNextPoint(points[i].x, points[i].y, points[i].z);
                // unsigned char color[3] = {points[i].r,points[i].g,points[i].b};
                colors->InsertNextTuple3(points[i].r,points[i].g,points[i].b);
                //float intensitiy = points[i].intensity;
                //intensities->InsertNextTupleValue(&intensity);
                //float snr = points[i].snr;
                //snrs->InsertNextTupleValue(&snrs);
            }

            // fclose(fp);

            // fp = fopen("lout","w");
            vtkCellArray* cells = vtkCellArray::New();
            for (size_t i=min_hz;i<max_hz;i++) {
                bool building = false;
                vtkTriangleStrip* triangleStrip = NULL;
                for (size_t j=min_vt;j<=max_vt;j++) {
                    LeicaScanPoint l1,l2;
                    if (hasPoint(i,j,l1) && hasPoint(i+1,j,l2) && 
                            ((edge_threshold<0) ||(dist(l1,l2)<edge_threshold))) {
                        LeicaScanPoint l1p,l2p;
                        if (building && hasPoint(i,j-1,l1p) && hasPoint(i+1,j-1,l2p)) {
                            if ((edge_threshold>0) && ((dist(l1,l1p) > edge_threshold) || (dist(l2,l2p) > edge_threshold))) {
                                if (triangleStrip->GetPointIds()->GetNumberOfIds()>=3) {
                                    cells->InsertNextCell(triangleStrip);
                                }
                                triangleStrip = vtkTriangleStrip::New();
                            }
                            // fprintf(fp,"%d %d\n%d %d\n\n\n",l1.hz,l1.vt,l1p.hz,l1p.vt);
                            // fprintf(fp,"%d %d\n%d %d\n\n\n",l2.hz,l2.vt,l2p.hz,l2p.vt);
                            // fprintf(fp,"%d %d\n%d %d\n\n\n",l2p.hz,l2p.vt,l1.hz,l1.vt);
                        }
                        if (!building) {
                            triangleStrip = vtkTriangleStrip::New();
                        }
                        triangleStrip->GetPointIds()->InsertNextId(l2.id);
                        triangleStrip->GetPointIds()->InsertNextId(l1.id);
                        // fprintf(fp,"%d %d\n%d %d\n\n\n",l1.hz,l1.vt,l2.hz,l2.vt);
                        building = true;
                    } else {
                        if (building && (triangleStrip->GetPointIds()->GetNumberOfIds()>=3)) {
                            cells->InsertNextCell(triangleStrip);
                        }
                        building = false;
                    }
                }
                if (building && (triangleStrip->GetPointIds()->GetNumberOfIds()>=3)) {
                    cells->InsertNextCell(triangleStrip);
                }
            }
            // fclose(fp);
            vtkPolyData* volume = vtkPolyData::New();
            volume->SetPoints(vpoints);
            volume->SetStrips(cells);
            volume->GetPointData()->SetScalars(colors);

#if VTK_MAJOR_VERSION <= 5
            volume->Update();
#endif

            vtkNew<vtkTriangleFilter> triangulation;
            triangulation->SetInputData(volume);
            triangulation->Update();

            vtkNew<vtkDecimatePro> decimate;
            if (vtk_decimate>0) {
                decimate->SetInputData(triangulation->GetOutput());
                decimate->SetTargetReduction(vtk_decimate);
                decimate->Update();
            }

            vtkPolyData * out = (vtk_decimate>0)?decimate->GetOutput():triangulation->GetOutput();

            if (reconstruct_volume) {

                printf("After triangulation (%.2f): %lld points, %lld polys, %lld vertices, %lld cells\n",vtk_decimate,
                        out->GetNumberOfPoints(), out->GetNumberOfPolys(), out->GetNumberOfVerts(), out->GetNumberOfCells());
                vtkNew<vtkIdFilter> idFilter;
                idFilter->SetInputData(out);
                idFilter->SetPointIdsArrayName("ids");
                idFilter->SetPointIds(true);
                idFilter->SetCellIds(false);
                // Available for vtk>=8.3:
                //idFilter.SetPointIdsArrayName("ids");
                //idFilter.SetCellIdsArrayName("ids");
                idFilter->Update();

#if 0
                vtkNew<vtkAdaptiveSubdivisionFilter> subdiv;
                subdiv->SetInputData(idFilter->GetOutput());
                subdiv->SetMaximumEdgeLength(0.5);
                subdiv->SetMaximumTriangleArea(1.0);
                //subdiv->SetMaximumNumberOfPasses(1);
                subdiv->Update();
#endif


                vtkNew<vtkFeatureEdges> edges;
                edges->SetInputData(idFilter->GetOutput());
                edges->BoundaryEdgesOn();
                edges->ManifoldEdgesOff();
                edges->NonManifoldEdgesOff();
                edges->FeatureEdgesOff();
                edges->Update();



                origin_id = out->GetNumberOfPoints();
                out->GetPoints()->InsertNextPoint(volume_origin_x,volume_origin_y,volume_origin_z);

                vtkPolyData* edgeData = edges->GetOutput();
                vtkDataArray *idArray = edgeData->GetPointData()->GetArray("ids");
                vtkCellArray* lines = edgeData->GetLines();
                const vtkIdType* indices;
                vtkIdType numberOfPoints;
                unsigned int lineCount = 0;
                for (lines->InitTraversal(); lines->GetNextCell(numberOfPoints, indices); lineCount++) {
                    for (vtkIdType i = 0; i < numberOfPoints; ++i) {
                        vtkTriangle* triangle = vtkTriangle::New();
                        triangle->GetPointIds()->SetNumberOfIds(3);
                        triangle->GetPointIds()->SetId(0,origin_id);
                        triangle->GetPointIds()->SetId(1,idArray->GetTuple1(indices[(i+1)%numberOfPoints]));
                        triangle->GetPointIds()->SetId(2,idArray->GetTuple1(indices[i]));
                        out->GetPolys()->InsertNextCell(triangle);
                    }
                }
#if VTK_MAJOR_VERSION <= 5
                out->Update();
#endif

                if (save_vtk && !filename.empty()) {
                    vtkNew<vtkPolyDataWriter> vwriter;
                    vwriter->SetFileName((filename+"-free.vtk").c_str());
#if VTK_MAJOR_VERSION <= 5
                    vwriter->SetInput(out);
#else
                    vwriter->SetInputData(out);
#endif
                    vwriter->Write();
                    printf("Saved vtk free mesh in %s-free.vtk\n",filename.c_str());
                }

                if (save_ply && !filename.empty()) {
                    vtkNew<vtkPLYWriter> writer2;
                    writer2->SetArrayName("colour");
                    writer2->SetFileName((filename+"-free.ply").c_str());
#if VTK_MAJOR_VERSION <= 5
                    writer2->SetInput(out);
#else
                    writer2->SetInputData(out);
#endif
                    writer2->Write();
                    printf("Saved ply mesh in %s-free.ply\n",filename.c_str());
                }

                return true;

            }


            int id_offset = 0;
            if (append) {
                assert(m_mesh || s_mesh);
                if (m_mesh) {
                    if (m_color) {
                        assert(m_mesh->vertices.size() == m_color->vertex_colors.size());
                    }
                    id_offset = m_mesh->vertices.size();
                } else {
                    id_offset = s_mesh->vertices.size();
                }
            } else {
                if (m_mesh) {
                    m_mesh->vertices.clear();
                    m_mesh->vertex_normals.clear();
                    m_mesh->faces.clear();
                } 
                if (m_color) {
                    m_color->vertex_colors.clear();
                }
                if (s_mesh) {
                    s_mesh->vertices.clear();
                    s_mesh->triangles.clear();
                }
            }
            if (m_mesh || s_mesh) {
                vtkPoints * o_pts = out->GetPoints();
                vtkDataArray * o_array = out->GetPointData()->GetScalars();
                printf("After decimation: %d points (%d tuples %d scalar dimensions), %d cells\n",
                        int(o_pts->GetNumberOfPoints()), int(o_array->GetNumberOfTuples()),
                        int(o_array->GetNumberOfComponents()), int(out->GetNumberOfCells()));
                for (int i=0;i<o_pts->GetNumberOfPoints();i++) {
                    geometry_msgs::msg::Point P; 
                    P.x = o_pts->GetPoint(i)[0];
                    P.y = o_pts->GetPoint(i)[1];
                    P.z = o_pts->GetPoint(i)[2];
                    if (m_mesh) {
                        m_mesh->vertices.push_back(P);
                    }
                    if (m_color && (o_array->GetNumberOfComponents()==3)) {
                        
                        double * tuple = o_array->GetTuple(i);
                        std_msgs::msg::ColorRGBA rgb;
                        rgb.r = tuple[0];
                        rgb.g = tuple[1];
                        rgb.b = tuple[2];
                        rgb.a = 255;
                        m_color->vertex_colors.push_back(rgb);
                    }
                    if (s_mesh) {
                        s_mesh->vertices.push_back(P);
                    }
                }
                for (int i=0;i<out->GetNumberOfCells();i++) {
                    vtkCell * c = out->GetCell(i);
                    // printf("Cell %d: %d vertices\n", int(i),int(c->GetNumberOfPoints()));
                    if (c->GetNumberOfPoints()!=3) {
                        std::cerr << "ERROR: Face " << i << " is not triangulated." << std::endl;
                        continue;
                    }
#ifdef HAS_MESH_MSGS
                    mesh_msgs::msg::MeshTriangleIndices m_mti;
                    for (int j=0;j<3;j++) {
                        m_mti.vertex_indices[j] = c->GetPointId(j)+id_offset;
                    }
                    if (m_mesh) {
                        m_mesh->faces.push_back(m_mti);
                    }
#endif
                    shape_msgs::msg::MeshTriangle s_mti;
                    for (int j=0;j<3;j++) {
                        s_mti.vertex_indices[j] = c->GetPointId(j)+id_offset;
                    }
                    if (s_mesh) {
                        s_mesh->triangles.push_back(s_mti);
                    }
                }
            }

            if (save_vtk && !filename.empty()) {
                vtkNew<vtkPolyDataWriter> writer;
                writer->SetFileName((filename+"-triangle.vtk").c_str());
#if VTK_MAJOR_VERSION <= 5
                writer->SetInput(out);
#else
                writer->SetInputData(out);
#endif
                writer->Write();
                printf("Saved vtk mesh in %s-triangle.vtk\n",filename.c_str());
            }

            if (save_ply && !filename.empty()) {
                vtkNew<vtkPLYWriter> writer2;
                writer2->SetArrayName("colour");
                writer2->SetFileName((filename+"-triangle.ply").c_str());
#if VTK_MAJOR_VERSION <= 5
                writer2->SetInput(out);
#else
                writer2->SetInputData(out);
#endif
                writer2->Write();
                printf("Saved ply mesh in %s-triangle.ply\n",filename.c_str());
            }

            
            return true;
#endif
        }

};


#endif // LEICA_SDB_PARSER_H
