/***************************************************************
 * Name:      NoNameRc_DesktopMain.h
 * Purpose:   Defines Application Frame
 * Author:    NoNameRc_TEAM ()
 * Created:   2016-09-08
 * Copyright: NoNameRc_TEAM ()
 * License:
 **************************************************************/

#ifndef NoNameRc_DESKTOPMAIN_H
#define NoNameRc_DESKTOPMAIN_H
#include <wx/stdpaths.h>
#include <wx/filename.h>
#include <wx/fileconf.h>
#include <wx/dcclient.h>


#define LCD_W 128
#define LCD_H 64
#define SPLASHLENGHT 1026


//(*Headers(NoNameRc_DesktopFrame)
#include <wx/menu.h>
#include <wx/listbox.h>
#include <wx/statbox.h>
#include <wx/panel.h>
#include <wx/button.h>
#include <wx/frame.h>
#include <wx/statusbr.h>
//*)

class NoNameRc_DesktopFrame: public wxFrame
{
public:

    NoNameRc_DesktopFrame(wxWindow* parent,wxWindowID id = -1);
    virtual ~NoNameRc_DesktopFrame();


private:

    //(*Handlers(NoNameRc_DesktopFrame)
    void OnQuit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);
    void OnButton1Click(wxCommandEvent& event);
    void OnavrdudeSelected(wxCommandEvent& event);
    void OnProgrammerSelected(wxCommandEvent& event);
    void OnreadmodelsSelected(wxCommandEvent& event);
    void OnreadfirmwareSelected(wxCommandEvent& event);
    void OnEcrirelesFuseesSelected(wxCommandEvent& event);
    void OnEcrirelebootloaderSelected(wxCommandEvent& event);
    void OnWriteModelToRadioSelected(wxCommandEvent& event);
    void OnWriteFirmwareToRadioSelected(wxCommandEvent& event);
    void OnButton1Click1(wxCommandEvent& event);
    void OnSimulateurClick2(wxCommandEvent& event);
    void OnATMEGA2560CompilerSelected(wxCommandEvent& event);
    void LoadConfig(wxString temp);
    void SaveConfig();
    void RestoreDefaultSplash();
    void OnClose(wxCloseEvent& event);
    void OnListBoxConfigDClick(wxCommandEvent& event);
    void OnMenuNewconfigSelected(wxCommandEvent& event);
    void OnMenuDeleteActiveConfigSelected(wxCommandEvent& event);
    void OnNexStepRc_M2560_docsSelected(wxCommandEvent& event);
    void OnMenuJQ6500_PCBSelected(wxCommandEvent& event);
    void OnMenuVoice_tutoSelected(wxCommandEvent& event);
    void OnMenuVOICE_AUDIO_PCBSelected(wxCommandEvent& event);
    void OnWIN_FIRMWARESelected(wxCommandEvent& event);
    void OnMenuAUTRES_LINKSSelected(wxCommandEvent& event);
    void OnMenuMousseSelected(wxCommandEvent& event);
    void DrawLbmSplash();
    void OnPanelSplashPaint(wxPaintEvent& event);
    void OnButtonSplashDefaultClick(wxCommandEvent& event);
    void OnButtonPersoClick(wxCommandEvent& event);
    //*)

    //(*Identifiers(NoNameRc_DesktopFrame)
    static const long ID_STATICBOXSPLASH;
    static const long ID_STATICBOXCONFIG;
    static const long ID_BUTTON1;
    static const long ID_LISTBOXCONFIG;
    static const long ID_SPLASH;
    static const long ID_BUTTONPERSO;
    static const long ID_BUTTONSPLASHDEFAULT;
    static const long ID_PANEL1;
    static const long ID_MENUITEMNEWCONFIG;
    static const long ID_MENUDELETEACTIVECONFIG;
    static const long idMenuQuit;
    static const long ID_MENUITEM1;
    static const long ID_MENUITEM3;
    static const long ID_MENUITEM5;
    static const long ID_MENUITEM4;
    static const long ID_MENUITEM6;
    static const long ID_MENUITEM9;
    static const long ID_MENUITEM10;
    static const long ID_MENUITEM7;
    static const long ID_MENUCOMPILOMATIC;
    static const long ID_MENUITEM14;
    static const long ID_MENUITEM8;
    static const long ID_MENUITEM11;
    static const long ID_MENUITEM12;
    static const long ID_MENUITEM13;
    static const long ID_MENUITEM16;
    static const long ID_MENUITEM15;
    static const long ID_MENUITEM2;
    static const long idMenuAbout;
    static const long ID_STATUSBAR1;
    //*)

    //(*Declarations(NoNameRc_DesktopFrame)
    wxButton* ButtonPerso;
    wxMenuItem* MenuItem7;
    wxMenuItem* MenuDeleteActiveConfig;
    wxPanel* PanelSplash;
    wxMenuItem* MenuItem5;
    wxButton* ButtonSplashDefault;
    wxMenu* MenuHtmlDoc;
    wxMenu* MenuItem15;
    wxMenu* Menu3;
    wxButton* Button1;
    wxMenuItem* MenuItem4;
    wxMenuItem* MenuItem14;
    wxMenuItem* MenuItem11;
    wxStaticBox* StaticBoxSplash;
    wxPanel* Panel1;
    wxMenuItem* MenuItem13;
    wxMenu* MenuItem8;
    wxMenuItem* MenuItem10;
    wxMenuItem* MenuItem12;
    wxMenuBar* MenuBar_main;
    wxMenuItem* MenuItem3;
    wxMenu* Menu7;
    wxListBox* ListBoxConfig;
    wxMenuItem* Menu5;
    wxStatusBar* StatusBar_main;
    wxStaticBox* StaticBoxConfig;
    wxMenuItem* ATMEGA2560Compiler;
    wxMenuItem* MenuItem6;
    wxMenuItem* MenuItem16;
    wxMenuItem* MenuItem9;
    wxMenuItem* MenuNewconfig;
    wxMenu* Menu4;
    //*)

    wxFileConfig* configFile;
    wxString Ini_Filename;
    wxString Profil;
    wxArrayString SavedConfig;

    wxString defaut;

    DECLARE_EVENT_TABLE()
};

#endif // NoNameRc_DESKTOPMAIN_H

