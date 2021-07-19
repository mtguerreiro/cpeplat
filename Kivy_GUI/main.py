
import kivy
import csv
import numpy as np
from kivy.app import App
from kivy.uix.popup import Popup
from kivy.core.window import Window
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.relativelayout import RelativeLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.tabbedpanel import TabbedPanel
from kivy.uix.widget import Widget
from kivy.properties import ObjectProperty, StringProperty, BooleanProperty
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
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
        printlog("data object created file :"+self.filepath)
        
    def read_oscilloscope_file(self):
 
        with open(self.filepath, "r") as csvfile:
            reader= csv.reader(csvfile,delimiter=' ')
            for row in reader:
                if (self.row_numbers==0):
                    self.row_numbers=len(row)
                new_row=list()
                for number in row:
                    new_row.append(float(number))
                    
                self.data.append(new_row)
            self.col_number=len(new_row)
        printlog("executei o read oscill")
        
    def updateExtension(self):
        self.extension =self.filepath.split('.')[-1]
    def getDataList(self,col_n,scale):
        column = (np.array(self.data)[:,col_n]*scale).tolist()
        return column
    def getColNum(self):
      return self.col_number
  
class ButtonRel(RelativeLayout):
    def __init__(self, **kwargs):
        super(ButtonRel, self).__init__(**kwargs)
        self.size=(300,300)     
        self.pos=(300,300)
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
                    
                #print(self.data1.getDataList())
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
     
    def update_graphs(self):
        test=19
    
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
class Tabs(TabbedPanel):
    inspector.create_inspector(Window,FileHeader)
    source_forms=ObjectProperty(None)
    ini=BooleanProperty(False)
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        if self.ini==False:
            self.source_forms=[]
            self.source_num=0
            self.y_init=.920
            self.ini=True  
            self.add_source()
            self.new_fig=MyFigure(figure=plt.gcf())
    
                
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
        for source in self.source_forms:
            for index,row in enumerate(source.getInputs()):
                scale=float(row.ids.scale_input.text)
                if (row.ids.checkbox_show._get_active()):
                    plt.plot(source.getDataListParent(index,scale),label=(row.ids.label_input.text))
        plt.legend(loc='upper left', ncol=1, shadow=True, fancybox=True)
        self.new_fig=MyFigure(figure=plt.gcf()) 
        self.ids.form_layout.add_widget(self.new_fig)
        self.ini=False
    
    pass

        
        
    
    
class ExperimentApp(App):
    def build(self):
        return Tabs()
    
if __name__ == '__main__':
    ExperimentApp().run()
 