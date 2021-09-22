import c4d
import os
PLUGIN_ID = 1038428
VERSION = 1.4


class Modeling(object):

    @staticmethod
    def GetWorldRotation(doc):
        mat = c4d.modules.snap.GetWorkplaneMatrix(doc, None)
        rot = c4d.utils.MatrixToHPB(mat)
        return rot

    @staticmethod
    def mix_vector(vector_list):
        if not len(vector_list):
            return None

        final_vector = c4d.Vector()
        for vec in vector_list:
            final_vector += vec

        final_vector = c4d.Vector(final_vector.x / len(vector_list),
                                  final_vector.y / len(vector_list),
                                  final_vector.z / len(vector_list))

        return final_vector

    @staticmethod
    def GlobalToLocal(obj, global_pos):
        obj_mg = obj.GetMg()
        return ~obj_mg * global_pos

    @staticmethod
    def LocalToGlobal(obj, local_pos):
        obj_mg = obj.GetMg()
        return obj_mg * local_pos

    @staticmethod
    def GetPointsGlobal(obj):
        all_points_pos = obj.GetAllPoints()
        for i in range(len(all_points_pos)):
            all_points_pos[i] = Modeling.LocalToGlobal(obj, all_points_pos[i])
        return all_points_pos

    @staticmethod
    def SetPointsGlobal(obj, global_point):
        points = obj.GetAllPoints()
        
        for i in range(len(points)):    
          points[i] = Modeling.GlobalToLocal(obj, global_point[i])

        obj.SetAllPoints(points)
        obj.Message(c4d.MSG_UPDATE)

    @staticmethod
    def GetTangentGlobal(obj):
        pts_local = obj.GetAllPoints()
        global_tangent = list()

        for pt_id, pt_local_pos in enumerate(pts_local):
            local_tangent = obj.GetTangent(pt_id)
            local_tangent["vl"] = Modeling.LocalToGlobal(obj, local_tangent["vl"] + pt_local_pos)
            local_tangent["vr"] = Modeling.LocalToGlobal(obj, local_tangent["vr"] + pt_local_pos)
            global_tangent.append(local_tangent)
        return global_tangent

    @staticmethod
    def SetTangentGlobal(obj, global_tangent_list):
        pt_count = obj.GetPointCount()
        pts = obj.GetAllPoints()
        local_tangent_list = list()

        # Global to Local tangent
        for i in range(pt_count):
            vl_local = Modeling.GlobalToLocal(obj, global_tangent_list[i]["vl"]) - pts[i]
            vr_local = Modeling.GlobalToLocal(obj, global_tangent_list[i]["vr"]) - pts[i]
            local_tangent_list.append([vl_local, vr_local])

        # Set Tangent
        for tangent_id, tangent in enumerate(local_tangent_list):
            obj.SetTangent(tangent_id, tangent[0], tangent[1])
        
    @staticmethod
    def SetGlobalRotation(obj, rot):
        all_children = obj.GetChildren()
        buffer_data = []
        for child in all_children:
            buffer_child = child.GetMg()
            buffer_data.append(buffer_child)

        old_pos = Modeling.GetPointsGlobal(obj)
        old_tangent = None
        if isinstance(obj, c4d.SplineObject):
            if obj.GetInterpolationType() == c4d.SPLINETYPE_BEZIER:
                old_tangent = Modeling.GetTangentGlobal(obj)

        bottom = None
        for pt in old_pos:
            buffer_bottom = pt.y
            if bottom is None or bottom > buffer_bottom:
                bottom = buffer_bottom
        if not bottom:
            return
        moyenne = Modeling.mix_vector(old_pos)
        m = obj.GetMg()
        pos = moyenne
        pos.y = bottom
        scale = c4d.Vector( m.v1.GetLength(),
                            m.v2.GetLength(),
                            m.v3.GetLength())

        m = c4d.utils.HPBToMatrix(rot)

        m.off = pos
        m.v1 = m.v1.GetNormalized() * scale.x
        m.v2 = m.v2.GetNormalized() * scale.y
        m.v3 = m.v3.GetNormalized() * scale.z

        obj.SetMg(m)
        
        Modeling.SetPointsGlobal(obj, old_pos)
        if old_tangent:
            Modeling.SetTangentGlobal(obj, old_tangent)

        for child_index in range(len(all_children)):
            child = all_children[child_index]
            buffer_child = buffer_data[child_index]
            child.SetMg(buffer_child)

    @staticmethod
    def resetSize(list_obj, doc):
        settings = c4d.BaseContainer()
        settings[c4d.MDATA_RESETSYSTEM_COMPENSATE] = True
        settings[c4d.MDATA_RESETSYSTEM_RECURSIVE] = True

        c4d.utils.SendModelingCommand(command=c4d.MCOMMAND_RESETSYSTEM,
                                      list=list_obj,
                                      mode=c4d.MODELINGCOMMANDMODE_ALL,
                                      bc=settings,
                                      doc=doc)


class UILauncher_Main(c4d.plugins.CommandData):
    dialog = None
    accepted = [c4d.Onull,
                c4d.Obezier,
                c4d.Osweep,
                c4d.Oloft,
                c4d.Olathe,
                c4d.Oextrude,
                c4d.Osds,
                c4d.Oboole,
                c4d.Oinstance,
                c4d.Osymmetry,
                c4d.Ometaball,
                c4d.Oconnector]

    def Execute(self, doc):
        self.action()
        return True

    def action(self):
        doc = c4d.documents.GetActiveDocument()
        objs = doc.GetActiveObjects(c4d.GETACTIVEOBJECTFLAGS_CHILDREN)

        doc.StartUndo()

        #Reset Size
        for obj in objs:
            doc.AddUndo(c4d.UNDOTYPE_CHANGE, obj)
        Modeling.resetSize(objs, doc)

        #We firstly reset pos of each points objects
        for obj in reversed(objs):
            if isinstance(obj, c4d.PointObject) and not self.is_first_child_of_sweep(obj):
                doc.AddUndo(c4d.UNDOTYPE_CHANGE, obj)
                rotation = Modeling.GetWorldRotation(doc)
                Modeling.SetGlobalRotation(obj, rotation)
                obj.Message(c4d.MSG_UPDATE)
                c4d.EventAdd()

        for obj in reversed(objs):
            if obj.GetType() in self.accepted:
                # get children matrice
                obj_under = obj.GetChildren()
                all_obj_under = self.get_all_children(obj, obj.GetNext(), True)
                old_matrice = self.get_list_of_matrice(obj_under)

                #change_null obj
                doc.AddUndo(c4d.UNDOTYPE_CHANGE, obj)
                bottom = None
                all_pos = list()

                #Get the bottom of all
                for i in range(len(all_obj_under)):
                    mat = all_obj_under[i].GetMg()
                    all_pos.append(mat.off)
                    buffer_bottom = mat.off.y
                    if bottom is None or bottom > buffer_bottom:
                        bottom = buffer_bottom

                if not all_obj_under:
                    continue
                    
                #get the average and set new matrice pos
                moyenne = Modeling.mix_vector(all_pos)
                if moyenne is None:
                    moyenne = mat.off
                else:
                    moyenne.y = bottom
                m = obj.GetMg()
                m.off = moyenne

                doc.AddUndo(c4d.UNDOTYPE_CHANGE, obj)
                obj.SetMg(m)
                obj.SetAbsRot(c4d.Vector())
                obj.SetAbsScale(c4d.Vector(1))
                obj.Message(c4d.MSG_UPDATE)

                # set children matrice
                self.set_list_of_matrice(obj_under, old_matrice, doc)

        c4d.EventAdd()
        doc.EndUndo()

    def is_first_child_of_sweep(self, obj):
        if not obj.GetUp():
            return False

        obj_up = obj.GetUp()
        if obj_up.GetDown() != obj:
            return False

        if obj_up.GetDown().GetDown():
            return False

        if obj_up.CheckType(c4d.Osweep):
            return True
        elif obj_up.CheckType(c4d.Olathe):
            return True
        else:
            return False

    def set_list_of_matrice(self, obj_under, new_matrice, doc):
        for i in range(len(obj_under)):
            doc.AddUndo(c4d.UNDOTYPE_CHANGE, obj_under[i])
            obj_under[i].SetMg(new_matrice[i])
            obj_under[i].Message(c4d.MSG_UPDATE)

    def get_list_of_matrice(self, obj_list):
        buffer_pos = list()
        for obj in obj_list:
            buffer_pos.append(obj.GetMg())

        return buffer_pos

    def mix_vector(self, vector_list):
        if not len(vector_list):
            return None

        final_vector = c4d.Vector()
        for vec in vector_list:
            final_vector += vec

        final_vector = c4d.Vector(final_vector.x / len(vector_list),
                                  final_vector.y / len(vector_list),
                                  final_vector.z / len(vector_list))

        return final_vector

    def get_all_children(self, obj, next_obj, first=False):
        buffer_children = list()
        while obj and obj != next_obj:
            if not first:
                buffer_children.append(obj)
            buffer_children += self.get_all_children(obj.GetDown(), next_obj)
            obj = obj.GetNext()

        return buffer_children

if __name__ == "__main__":
    dir, file = os.path.split(__file__)
    bmp = c4d.bitmaps.BaseBitmap()
    bmp.InitWith(os.path.join(dir, "center_axis.png"))
    c4d.plugins.RegisterCommandPlugin(id=PLUGIN_ID, 
                                    str="06 - Bottom axis center - v" + str(VERSION),
                                    help="06 - Bottom axis center - v" + str(VERSION),
                                    info=0,
                                    dat=UILauncher_Main(), 
                                    icon=bmp)
