/************************************************************/
/*    NAME: Adam Pressel                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenRescue.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "GenRescue.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

GenRescue::GenRescue()
{
  // current posiiton
  m_nav_x = 0;
  m_nav_y = 0;

  // bools of first coord reading
  m_x = false;
  m_y = false;

  m_visit_radius = 5;

  m_curr_swimmer_list_size = 0;
  m_curr_swimmer_found_list_size = 0;
}

//---------------------------------------------------------
// Destructor

GenRescue::~GenRescue()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GenRescue::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
    if (key == "SWIMMER_ALERT")
    {

      string alert_str = msg.GetString();

      // double temp_swim_x_pos, temp_swim_y_pos;
      // string temp_swim_id;

      XYPoint temp_swim_pt = string2Point(alert_str);
      string temp_swim_id = temp_swim_pt.get_id();

      // temp_swim_pt.set_vertex(temp_swim_x_pos, temp_swim_y_pos, temp_swim_id);

      m_map_swimmers[temp_swim_id] = temp_swim_pt;
    }
    else if (key == "NAV_X")
    {
      // if (m_x == false)
      m_nav_x = msg.GetDouble();
      m_x = true;
    }
    else if (key == "NAV_Y")
    {
      // if (m_x == false)
      m_nav_y = msg.GetDouble();
      m_y = true;
    }
    else if (key == "FOUND_SWIMMER")
    {
      string alert_str = msg.GetString();
      string temp_swim_id = tokStringParse(alert_str, "id", ',', '=');
      m_set_swimmers_found.insert(temp_swim_id);
    }
    else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GenRescue::OnConnectToServer()
{
  registerVariables();
  Notify("CONNECTED", "true");

  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool GenRescue::Iterate()
{
  AppCastingMOOSApp::Iterate();

  vector<XYPoint> points_to_visit_list;
  bool gen_path = false;

  if (m_x == true && m_y == true)
  {
    // cout << m_x << endl;
    double current_x = m_nav_x;
    double current_y = m_nav_y;

    // populate the list of points that have not been found
    // int counter = 0;
    for (auto it = m_map_swimmers.begin(); it != m_map_swimmers.end(); it++)
    {
      string id = it->first;
      XYPoint point = it->second;
      if (m_set_swimmers_found.count(id) == 0)
      {
        points_to_visit_list.push_back(point);
        // counter++;
      }
    }

    // // DEBUG
    // XYSegList debug;
    // for (int i = 0; i < points_to_visit_list.size(); i++)
    // {
    //   debug.add_vertex(points_to_visit_list[i]);
    // }
    // string debug_str = debug.get_spec() + to_string(counter);
    // Notify("DEBUG_UPDATE", debug_str);

    // if there is a change in the target swimmers list or
    //             a change in the number of swimmers found by anyone
    if (m_curr_swimmer_list_size != points_to_visit_list.size() ||
        m_curr_swimmer_found_list_size != m_set_swimmers_found.size())
    {
      gen_path = true;
      m_curr_swimmer_list_size = points_to_visit_list.size();
      m_curr_swimmer_found_list_size = m_set_swimmers_found.size();
    }

    // begin TSP Problem
    XYSegList seg_list;
    if (gen_path)
    {
      vector<bool> sorted(points_to_visit_list.size(), false);
      for (int n = 0; n < points_to_visit_list.size(); n++)
      {
        double shortest_distance = -1;
        int short_index = 0;
        // for every point in the points to visit, calculate distance to current x_y
        for (int i = 0; i < points_to_visit_list.size(); i++)
        {
          double delta_x = abs(current_x - points_to_visit_list[i].x());
          double delta_y = abs(current_y - points_to_visit_list[i].y());
          double temp_distance = sqrt((delta_x * delta_x) + (delta_y * delta_y));
          if (!sorted[i])
          {
            if (shortest_distance < 0 || shortest_distance > temp_distance)
            {
              shortest_distance = temp_distance;
              short_index = i;
            }
          }
        }

        seg_list.add_vertex(points_to_visit_list[short_index].x(), points_to_visit_list[short_index].y());

        current_x = points_to_visit_list[short_index].x();
        current_y = points_to_visit_list[short_index].y();
        sorted[short_index] = true;
      }
      string updates_str = "points = ";
      updates_str += seg_list.get_spec();
      Notify("SURVEY_UPDATE", updates_str);
    }
  }

  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open
// Checks for visit radius

bool GenRescue::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if (param == "foo")
    {
      handled = true;
    }
    else if (param == "bar")
    {
      handled = true;
    }
    else if (param == "visit_radius")
    {
      m_visit_radius = stoi(value);
      handled = true;
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void GenRescue::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
  Register("VISIT_POINT", 0); // Made to run on vehical DB, must ALIAS VISIT_POINT_VEHICAL -> VISIT_POINT
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("GENPATH_REGENERATE", 0);
  Register("SWIMMER_ALERT", 0);
  Register("FOUND_SWIMMER", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool GenRescue::buildReport()
{
  m_msgs << "============================================" << endl;

  m_msgs << "Size of swimmer map: " << m_map_swimmers.size() << endl;
  m_msgs << "Size of swimmer found set: " << m_set_swimmers_found.size() << endl;
  m_msgs << "Size of current swimmer found list: " << m_curr_swimmer_found_list_size << endl;
  m_msgs << "Size of current swimmer target list: " << m_curr_swimmer_list_size << endl;

  return (true);
}
