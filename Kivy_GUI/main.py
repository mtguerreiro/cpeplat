
import kivy
import csv
import numpy as np
import importlib.util
inter_spec = importlib.util.spec_from_file_location("interface.py", "../cpeplat/interface.py")
interface = importlib.util.module_from_spec(inter_spec)
inter_spec.loader.exec_module(interface)
interface.Commands()

from kivy.app import App
from kivy.uix.popup import Popup
from kivy.core.window import Window
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.textinput import TextInput
from kivy.uix.tabbedpanel import TabbedPanel
from kivy.uix.tabbedpanel import TabbedPanelItem
from kivy.uix.widget import Widget
from kivy.properties import ObjectProperty, StringProperty, BooleanProperty
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvas
from kivy.modules import inspector
import matplotlib.pyplot as plt


Window.size = (1000, 600)

def printlog(message):
    with open('./log.txt','a') as f: f.write(message+"\n")


class DataObject():
    def __init__(self,file_in=''):
        self.data=list()
        self.scale=1;
        self.filepath=file_in
        self.col_number=0;
        self.extension=''
        self.row_numbers=0
        self.label_std=['t','Vin','Vin_buck','Vo','Vo_buck','IL','IL_avg','u']
        printlog("data object created file :"+self.filepath)
        
    def read_oscilloscope_file(self):
        new_list=[]
        with open(self.filepath, "r") as csvfile:
            reader= csv.reader(csvfile,delimiter=' ')
            for row in reader:
                if (self.row_numbers==0):
                    self.row_numbers=len(row)
                new_row=list()
                for number in row:
                    new_row.append(float(number))
                    
                new_list.append(new_row)
            self.col_number=len(new_row)
        self.data=new_list.copy()
    def read_c2000_csv(self):
        new_list=[]
        with open(self.filepath, "r") as csvfile:
            reader= csv.reader(csvfile,delimiter=',')
            next(reader)
            for row in reader:
                if (self.row_numbers==0):
                    self.row_numbers=len(row)
                new_row=list()
                for number in row:
                    new_row.append(float(number))                    
                new_list.append(new_row)
            
            self.col_number=len(new_row)
        self.data=new_list.copy()
        
    def updateExtension(self):
        self.extension =self.filepath.split('.')[-1]
    def getDataList(self,col_n,scale):
        np_data = (np.array(self.data)*scale)     
        np_columns=np_data[:,col_n]
        columns=np_columns.tolist()
        return columns
    
    def getColNum(self):
      return self.col_number
  

class ChooseInputs(GridLayout):
    def __init__(self,**kwargs):
        super(ChooseInputs,self).__init__(**kwargs)
    
    pass

class InputsHeader(BoxLayout):
    def __init__(self,**kwargs):
        super(InputsHeader,self).__init__(**kwargs)
        self.size_hint_y=0.025
    pass


class FileHeader(BoxLayout):
    def __init__(self,**kwargs):
        super(FileHeader,self).__init__(**kwargs)
        self.spacing=10
        self.pos_hint={'y':0.800,'left':0}
        self.id_num=0
        self.input_rows=list()
        self.input_num=0
        self.add_widget(InputsHeader(),0)
        self.source_path=self.ids.get_file.text
        self.data_obj= DataObject()

     
    file_path = StringProperty("No file chosen")
    the_popup = ObjectProperty(None)
    
    def getInputs(self):
        return self.input_rows
                        
    def read_file(self): 
        printlog(self.data_obj.filepath)
        self.data_obj.updateExtension()
        
        printlog("extension="+self.data_obj.extension)
        if (self.data_obj.extension =='dat'): #add options for when files are different
            try:
                self.data_obj.read_oscilloscope_file()
                printlog("number of colums:"+str(self.data_obj.col_number))
                for i in range(0,self.data_obj.col_number):
                    self.add_input()
                    
                #self.data1.getDataList())
            except ValueError as ve:
                printlog(ve)

        
    def add_input(self):
        printlog("i have added an input "+str(self.input_num))
        if self.input_num<self.data_obj.getColNum():      
            self.pos_hint['y']=(self.pos_hint['y']-0.030)
            self.size_hint_y=self.size_hint_y+0.030
            self.input_rows.append(ChooseInputs())
            self.input_rows[self.input_num].id="input_"+str(self.input_num)
            self.input_rows[self.input_num].ids.col_num.text=str(self.input_num+1)
            self.add_widget(self.input_rows[self.input_num],0)
            self.input_num=self.input_num+1

    def load_text_input(self):        
        self.data_obj=DataObject(self.ids.get_file.text)
        self.read_file()

    def getDataListParent(self,index,scale):
        return self.data_obj.getDataList(index,scale)       

    def open_popup(self):
        self.the_popup = FileChoosePopup(load=self.load)
        self.the_popup.open()
    def load(self, selection):
        self.file_path = str(selection[0])  
        self.the_popup.dismiss()
        printlog("FILEPATH From selection is: "+self.file_path)
        # check for non-empty list i.e. file selected
        if self.file_path:
            self.ids.get_file.text = self.file_path
        self.data_obj=DataObject(self.file_path)

    def remove(self):
        #self.add_input()
        pass

    
class MyFigure(FigureCanvasKivyAgg):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.size_hint=(0.5, 0.65)
        self.pos_hint={'y': 0.15, 'right':1}


        
class FileChoosePopup(Popup):
    load = ObjectProperty()
    
    
class FormLayout(FloatLayout):

    pass

class Buck(TabbedPanelItem):
    ini=BooleanProperty(False)
    source_forms=ObjectProperty(None)
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.actual_data=DataObject('./Store/data_13v_pid.csv')
        self.sample_time=0.1
        
    def read_and_store(self):
        
        """read continuously data and store into a file, here follows the C2000 commands"""
        
        pass
    
    def upload(self):
        """update graph, will run cyclically"""
        self.read_file()
        time_axis=list()
        y_graph1 = []
        y_graph2 = []
        y_graph3 = []
        y_graph4 = []
        self.ids.plot_exp.set_plot_num(4)
        time_axis.append(self.getDataListParent([0],1/self.sample_time))
        y_graph1=self.getDataListParent([1,2],1).copy()
        y_graph2=self.getDataListParent([3,4],1).copy()
        y_graph3=self.getDataListParent([5,6],1).copy()
        y_graph4=self.getDataListParent([7],1).copy()
        print("size y_graph4: ",len(y_graph4))
        print("size get_data: ",len(self.getDataListParent([7],1)))
        self.ids.plot_exp.set_new_data([y_graph1,y_graph2,y_graph3,y_graph4])
        self.ids.plot_exp.draw_my_plot()
        pass
    
    def export_graph(self):
        """save graph to png"""

        pass
        
    def save_data_to_files(self):
        """save data from temporary file to a defined file"""
        
        pass
    
    def getDataListParent(self,index,scale):
        return self.actual_data.getDataList(index,scale)  
    
    def read_file(self):
        printlog(self.actual_data.filepath)
        self.actual_data.updateExtension()  
        printlog("extension="+self.actual_data.extension)
        if (self.actual_data.extension =='csv'): #add options for when files are different
                self.actual_data.read_c2000_csv()

                
                
class Analysis(TabbedPanelItem):
    ini=BooleanProperty(False)
    source_forms=ObjectProperty(None)
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.source_forms=[]
        self.source_num=0
        self.y_init=.920
        self.ini=True  
        self.new_fig=MyFigure(figure=plt.gcf())
        self.new_fig.id="graph"

                
    def add_source(self):

        self.source_forms.append(FileHeader())
        self.source_forms[self.source_num].pos_hint={'y':self.y_init-(self.source_num*0.15),'left':0}
        self.source_forms[self.source_num].id="src_"+str(self.source_num)
        self.source_forms[self.source_num].ids.src_n.text="Source "+str(self.source_num+1)+":"
        self.ids.form_layout.add_widget(self.source_forms[self.source_num],0)
        self.source_num+=1
        
    
    def remove_source(self):
        if(self.source_num)>1:
            try:
                self.ids.form_layout.remove_widget(self.source_forms[self.source_num-1])
                self.source_num-=1
            except:
                pass
                                           
    def update_sources(self):
        
        for source, ind in enumerate(self.source_forms):
                if (source.ids.get_file.text != ''):
                    source.data_list.append(DataObject(source.ids.getfile.text))
                    

                
    def update_graphs(self):   
        try:
            if not (self.ini):
                self.ids.form_layout.remove_widget(self.new_fig)
                printlog("removendo figura")
                plt.cla()
        except:
            pass
        x_data=list()
        legends=list()
        for source in self.source_forms:
            for index,row in enumerate(source.getInputs()):
                scale=float(row.ids.scale_input.text)
                if (row.ids.checkbox_show._get_active()):
                    x_data.append(source.getDataListParent(index,scale))
                    legends.append(row.ids.label_input.text)
        self.ids.plot_al.set_plot_num(1)
        self.ids.plot_al.set_title('Analysis Plot')
        self.ids.plot_al.set_new_data(x_data)
        self.ids.plot_al.set_new_legends(legends)
        
                
        #plt.plot(source.getDataListParent(index,scale),label=(row.ids.label_input.text))
        #plt.legend()
        #self.new_fig=MyFigure(figure=plt.gcf()) 
        #self.ids.form_layout.add_widget(self.new_fig)
        
        self.ids.plot_al.draw_my_plot()
        self.ini=False
    
    pass
    
class DrawPlot(BoxLayout):
    def __init__(self, **kwargs):
        super(DrawPlot, self).__init__(**kwargs)
        self.size_hint=(0.5,0.5)
        self.pos_hint={'x':0.5,'right':1} 
        self.plot_num=4;
        self.title='Experiment Plots'
        self.legends=(['V_in','V_in_Buck'], ['V_out','V_out_Buck'], ['I_l','I_l_AVG'] ,['Control'])
        self.labels=list()
        self.y_plot=list()      
        self.test=False



        
    def draw_my_plot(self):
        print("children are:")
        for child in self.children:
            print(child)
        self.clear_widgets()
        print("children are:")
        for child in self.children:
            print(child)
        fig, ax = plt.subplots(self.plot_num)
        #fig.cla()
        if self.plot_num>1:
            if self.title:
                fig.suptitle(self.title,fontsize=22) 

            for i in range(0,self.plot_num):

                ax[i].cla()
                ax[i].plot(self.y_plot[i])
                ax[i].legend(self.legends[i],loc='upper left', ncol=1, shadow=True, fancybox=True)
            canvas =  FigureCanvas(figure=fig)
                
        elif self.plot_num==1:
            for line in self.y_plot:
                ax.plot(line)
                ax.legend(self.legends,loc='upper left', ncol=1, shadow=True, fancybox=True)
            canvas =  FigureCanvas(figure=fig) 
            
       
        self.add_widget(canvas, 0)
        self.test=True
        return;
    def set_new_data(self,y_plot_new):
        new=[]

        for index, data in enumerate(y_plot_new):
            new.append(data)
            
        self.y_plot=new.copy() 
        #print(self.y_plot)
        return;
    def set_new_legends(self,new_legends):
        self.legends=new_legends
        return;
    def set_plot_num(self,num):
        self.plot_num=num
        return;
    def set_title(self,title):
        self.title=title
        return;
    def set_labels(self,labels):
        self.title=labels
        return;
        
class Tabs(TabbedPanel):
    inspector.create_inspector(Window,FileHeader)


        
        
    
    
class ExperimentApp(App):
    def build(self):
        return Tabs()
    
if __name__ == '__main__':
    ExperimentApp().run()
 