within ;
package HeatPumpTest

  model heatpumpTest

    import Project.*;
    import Modelica.*;
    import Modelica.Blocks.Interfaces.*;

    RealOutput M_air;
    RealOutput T_air;

    ThermoCycle.Components.Units.HeatExchangers.Hx1DInc condenser(
      redeclare package Medium1 = ThermoCycle.Media.R134a_CP,
      redeclare package Medium2 = ThermoCycle.Media.StandardWater,
      N=10,
      redeclare model Medium1HeatTransferModel =
          ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.MassFlowDependence,
      M_wall=5,
      Mdotnom_sf=0.5,
      Mdotnom_wf=0.1,
      A_sf=4,
      A_wf=2,
      Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
      V_sf=0.01,
      V_wf=0.001,
      Unom_sf=3000,
      steadystate_h_wf=false,
      pstart_wf=380000,
      Tstart_inlet_wf=345.15,
      Tstart_outlet_wf=310.15,
      Tstart_inlet_sf=310.15,
      Tstart_outlet_sf=310.15)
      annotation (Placement(transformation(extent={{10,16},{-16,42}})));

    ThermoCycle.Components.Units.HeatExchangers.Hx1DInc evaporator(
      redeclare package Medium1 = ThermoCycle.Media.R134a_CP,
      N=10,
      redeclare model Medium1HeatTransferModel =
          ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.MassFlowDependence,
      M_wall=10,
      Mdotnom_wf=0.1,
      A_wf=4,
      Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
      V_sf=0.002,
      V_wf=0.002,
      redeclare package Medium2 = Modelica.Media.Air.DryAirNasa,
      A_sf=20,
      Unom_sf=100,
      Mdotnom_sf=0.76,
      steadystate_h_wf=false,
      pstart_wf=230000,
      Tstart_inlet_wf=263.15,
      Tstart_outlet_wf=277.15,
      Tstart_inlet_sf=283.15,
      Tstart_outlet_sf=275.15)
      annotation (Placement(transformation(extent={{-12,-44},{14,-70}})));

    ThermoCycle.Components.Units.PdropAndValves.Valve valve(
      redeclare package Medium = ThermoCycle.Media.R134a_CP,
      Mdot_nom=0.1,
      UseNom=false,
      Afull=15e-7,
      Xopen=0.45,
      p_nom=380000,
      T_nom=308.15,
      DELTAp_nom=150000)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-40,-26})));

     ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor
                                                              compressor(
      epsilon_v=0.9,
      redeclare package Medium = ThermoCycle.Media.R134a_CP,
      V_s=8e-4,
      p_su_start=200000,
      p_ex_start=400000) annotation (Placement(transformation(
          extent={{-19,-18},{19,18}},
          rotation=180,
          origin={59,-16})));

    ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot airSideSource(
      redeclare package Medium = Modelica.Media.Air.DryAirNasa,
      Mdot_0=1.76,
      T_0=283.15)
      annotation (Placement(transformation(extent={{66,-88},{46,-68}})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SinkP airSideSink(redeclare
        package                                                                     Medium =
                 Modelica.Media.Air.DryAirNasa)
      annotation (Placement(transformation(extent={{-52,-74},{-72,-54}})));

    ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                 electricDrive
      annotation (Placement(transformation(extent={{28,-26},{8,-6}})));

    ThermoCycle.Components.Units.Tanks.ThermoclineStorage thermoclineStorage(
        replaceable package Medium = ThermoCycle.Media.StandardWater,
        N=10,
        V_tank=20,
        H_D=2.5,
        d_met=0.03,
        epsilon_p=1,
        Vlstart=15,
        p=101000,
        k_liq=500,
        k_wall=200,
        U_env_bottom=200,
        U_env_wall=200,
        U_env_top=200,
        pstart=101000,
        m_dot_su=0.02,
        m_dot_ex=0.02,
        Tstart_su=320,
        Tstart_ex=330)
       annotation (Placement(transformation(extent={{-8,-10},{8,10}},
          rotation=180,
          origin={-4,82})));

    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTpofEvaporator(redeclare
        package                                                                         Medium =
                 ThermoCycle.Media.R134a_CP)
      annotation (Placement(transformation(extent={{32,-58},{52,-38}})));

    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTpofCompressor(redeclare
        package                                                                         Medium =
                 ThermoCycle.Media.R134a_CP) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={50,16})));
    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTpofCondenser(redeclare
        package                                                                        Medium =
                 ThermoCycle.Media.R134a_CP) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-45,6})));
    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTpofHeatedWater(redeclare
        package                                                                          Medium =
                 ThermoCycle.Media.StandardWater)
      annotation (Placement(transformation(extent={{24,30},{44,50}})));

    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTpofTank(redeclare
        package                                                                   Medium =
                 ThermoCycle.Media.StandardWater)
      annotation (Placement(transformation(extent={{10,80},{30,100}})));
    ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump pumpTankWater(
      redeclare package Medium = ThermoCycle.Media.StandardWater,
      PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.ORCNext,
      PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.freq,
      hstart=2.27e5,
      M_dot_start=0.1,
      eta_is=0.9,
      epsilon_v=0.9,
      V_dot_max=0.05)
        annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={-40,56})));

    Modelica.Blocks.Sources.Step step(
      offset=50,
      height=5,
      startTime=50) annotation (Placement(transformation(extent={{-12,-2},{-4,6}})));

    ThermoCycle.Components.FluidFlow.Sensors.SensMdot sensMdot(
    replaceable package Medium = ThermoCycle.Media.StandardWater)
      annotation (Placement(transformation(extent={{40,80},{60,100}})));

    ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump pumpAir(
      redeclare package Medium = ThermoCycle.Media.StandardWater,
      PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.UD,
      PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.freq,
      hstart=2.27e5,
      M_dot_start=0.1,
      eta_is=0.9,
      epsilon_v=0.9,
      V_dot_max=0.05)
        annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={-50,118})));
    ThermoCycle.Components.Units.HeatExchangers.Hx1DInc evaporator1(
      redeclare package Medium1 = ThermoCycle.Media.Air_CP,
      N=10,
      redeclare model Medium1HeatTransferModel =
          ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.MassFlowDependence,
      M_wall=10,
      Mdotnom_wf=0.1,
      A_wf=4,
      Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
      V_sf=0.002,
      V_wf=0.002,
      redeclare package Medium2 = ThermoCycle.Media.StandardWater,
      A_sf=20,
      Unom_sf=100,
      Mdotnom_sf=0.76,
      steadystate_h_wf=false,
      pstart_wf=230000,
      Tstart_inlet_wf=263.15,
      Tstart_outlet_wf=277.15,
      Tstart_inlet_sf=283.15,
      Tstart_outlet_sf=275.15)
      annotation (Placement(transformation(extent={{-14,160},{12,134}})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot airSideSource1(
      redeclare package Medium = ThermoCycle.Media.Air_CP,
      Mdot_0=1.76,
      T_0=283.15)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=180,
          origin={-58,154})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SinkP airSideSink1(
                                                                  redeclare
        package                                                                     Medium =
                 ThermoCycle.Media.Air_CP)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=180,
          origin={58,162})));

    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensofAir(redeclare package Medium =
                 ThermoCycle.Media.Air_CP)
      annotation (Placement(transformation(extent={{20,166},{40,186}})));



   AixLib.Building.LowOrder.Multizone.Multizone multizone(
        buildingParam=Project.ResidentialBuilding.ResidentialBuilding_DataBase.ResidentialBuilding_base(),
     redeclare AixLib.Building.LowOrder.ThermalZone.ThermalZoneEquipped zone(
     redeclare
          AixLib.Building.LowOrder.BaseClasses.BuildingPhysics.BuildingPhysics
          buildingPhysics(
     redeclare
            AixLib.Building.Components.WindowsDoors.BaseClasses.CorrectionSolarGain.CorG_VDI6007
            corG)))
      annotation (Placement(transformation(extent={{-28,228},{26,278}})));
    AixLib.Building.Components.Weather.Weather weather(
      Outopt=1,
      Air_temp=true,
      Mass_frac=true,
      Sky_rad=true,
      Ter_rad=true,
      fileName=Modelica.Utilities.Files.loadResource(
        "modelica://AixLib/Resources/WeatherData/TRY2010_12_Jahr_Modelica-Library.txt"),
      tableName="wetter",
        Latitude=49.5,
        Longitude=8.5,
      SOD=AixLib.DataBase.Weather.SurfaceOrientation.SurfaceOrientationBaseDataDefinition(
       nSurfaces=5,
      name={"0.0", "90.0", "180.0", "270.0", "-1"},
      Azimut={180.0, -90.0, 0.0, 90.0, 0.0},
      Tilt={90.0, 90.0, 90.0, 90.0, 0.0}))
      annotation (Placement(transformation(extent={{-26,310},{4,330}})));
    Modelica.Blocks.Sources.CombiTimeTable tableInternalGains(
      tableOnFile=true,
      extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
      tableName="Internals",
      columns=2:4,
      fileName=Modelica.Utilities.Files.loadResource(
          "modelica://Project/ResidentialBuilding//InternalGains_ResidentialBuilding.mat"))
      annotation (Placement(transformation(extent={{82,260},{62,280}})));
  equation

    M_air = -evaporator1.outlet_fl1.m_flow;
    T_air = sensofAir.T;

    connect(evaporator.inlet_fl1, valve.OutFlow) annotation (Line(
        points={{-9,-52},{-40,-52},{-40,-35}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(airSideSource.flangeB, evaporator.inlet_fl2) annotation (Line(
        points={{47,-78},{20,-78},{20,-63},{10.8,-63}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(evaporator.outlet_fl2, airSideSink.flangeB) annotation (Line(
        points={{-8.8,-62.8},{-20,-62.8},{-20,-64},{-53.6,-64}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(electricDrive.shaft, compressor.flange_elc) annotation (Line(
        points={{26.6,-16},{46.3333,-16}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(evaporator.outlet_fl1, sensTpofEvaporator.InFlow) annotation (Line(
        points={{11,-52},{35,-52},{35,-52.8}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(sensTpofEvaporator.OutFlow, compressor.InFlow) annotation (Line(
          points={{49,-52.8},{69.7667,-52.8},{69.7667,-27.7}}, color={0,0,255}));
    connect(compressor.OutFlow, sensTpofCompressor.OutFlow) annotation (Line(
          points={{45.3833,-10},{45.2,-10},{45.2,9}}, color={0,0,255}));
    connect(sensTpofCompressor.InFlow, condenser.inlet_fl1) annotation (Line(
          points={{45.2,23},{30.5,23},{30.5,24},{7,24}}, color={0,0,255}));
    connect(condenser.outlet_fl1, sensTpofCondenser.OutFlow) annotation (Line(
          points={{-13,24},{-28,24},{-40.2,24},{-40.2,13}}, color={0,0,255}));
    connect(sensTpofCondenser.InFlow, valve.InFlow) annotation (Line(points={{-40.2,
            -1},{-40.2,-8.5},{-40,-8.5},{-40,-17}}, color={0,0,255}));
    connect(condenser.outlet_fl2, sensTpofHeatedWater.InFlow) annotation (Line(
          points={{6.8,34.8},{16.4,34.8},{16.4,35.2},{27,35.2}}, color={0,0,255}));
    connect(sensTpofHeatedWater.OutFlow, thermoclineStorage.portHotSF)
      annotation (Line(points={{41,35.2},{64,35.2},{64,75.9},{3.5,75.9}}, color={0,
            0,255}));
    connect(thermoclineStorage.portColdSF, sensTpofTank.InFlow) annotation (Line(
          points={{3.4,88.4},{8.7,88.4},{8.7,85.2},{13,85.2}}, color={0,0,255}));
    connect(thermoclineStorage.portHotPW, pumpTankWater.InFlow)
      annotation (Line(points={{-11.4,76},{-39.5,76},{-39.5,63.2}}, color={0,0,255}));
    connect(pumpTankWater.OutFlow, condenser.inlet_fl2)
      annotation (Line(points={{-32.6,50.4},{-32.6,35},{-12.8,35}}, color={0,0,255}));
    connect(step.y, electricDrive.f)
      annotation (Line(points={{-3.6,2},{-3.6,2},{17.6,2},{17.6,-6.6}}, color={0,0,127}));
    connect(sensTpofTank.OutFlow, sensMdot.InFlow)
      annotation (Line(points={{27,85.2},{29,85.2},{29,86},{46,86}},     color={0,0,255}));
    connect(pumpAir.OutFlow, thermoclineStorage.portColdPW) annotation (Line(points={{-42.6,112.4},{-42.6,
            100.2},{-11.7,100.2},{-11.7,88.35}}, color={0,0,255}));
    connect(sensMdot.OutFlow, evaporator1.inlet_fl2)
      annotation (Line(points={{54,86},{70,86},{70,141},{8.8,141}}, color={0,0,255}));
    connect(evaporator1.outlet_fl2, pumpAir.InFlow)
      annotation (Line(points={{-10.8,141.2},{-49.5,141.2},{-49.5,125.2}}, color={0,0,255}));
    connect(airSideSource1.flangeB, evaporator1.inlet_fl1)
      annotation (Line(points={{-49,154},{-30,154},{-30,152},{-11,152}}, color={0,0,255}));

    connect(evaporator1.outlet_fl1, sensofAir.InFlow)
      annotation (Line(points={{9,152},{16,152},{16,171.2},{23,171.2}}, color={0,0,255}));
    connect(sensofAir.OutFlow, airSideSink1.flangeB)
      annotation (Line(points={{37,171.2},{43.5,171.2},{43.5,162},{49.6,162}}, color={0,0,255}));

    connect(weather.SolarRadiation_OrientedSurfaces,multizone. radIn)
      annotation (Line(points={{-18.8,309},{-18.8,292.5},{-17.74,292.5},{-17.74,275.5}},
          color={255,128,0}));
    connect(weather.WeatherDataVector,multizone. weather) annotation (
       Line(points={{-11.1,309},{-11.1,294},{-5.32,294},{-5.32,276.5}},
                                                                  color={0,0,127}));
    connect(tableInternalGains.y,multizone. internalGains)
      annotation (Line(points={{61,270},{44,270},{44,269.25},{24.11,269.25}},
          color={0,0,127}));


    connect(T_air,multizone.ventilationTemperature[1]);
    connect(M_air,multizone.ventilationRate[1]);

   annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-240,-100},{220,360}})),
      experiment(StopTime=2000000),
      __Dymola_experimentSetupOutput,
      Icon(coordinateSystem(extent={{-240,-100},{220,360}})));
  end heatpumpTest;

  model TankTest

    ThermoCycle.Components.Units.Tanks.ThermoclineStorage thermoclineStorage(
        replaceable package Medium = ThermoCycle.Media.StandardWater,
        N=10,
        V_tank=20,
        H_D=2.5,
        d_met=0.03,
        epsilon_p=1,
        Vlstart=15,
        p=101000,
        k_liq=500,
        k_wall=200,
        U_env_bottom=200,
        U_env_wall=200,
        U_env_top=200,
        pstart=101000,
        m_dot_su=0.03,
        m_dot_ex=0.03,
        Tstart_su=300,
        Tstart_ex=300)
      annotation (Placement(transformation(extent={{-8,-2},{8,18}})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot source1(
      redeclare package Medium = ThermoCycle.Media.StandardWater,
      Mdot_0=0.1,
      T_0=320.15)
      annotation (Placement(transformation(extent={{-68,6},{-48,26}})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot source2(
      redeclare package Medium = ThermoCycle.Media.StandardWater,
      Mdot_0=0.1,
      T_0=298.15)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={48,0})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SinkMdot sink1(
      redeclare package Medium = ThermoCycle.Media.StandardWater,
      Mdot_0=0.1)
      annotation (Placement(transformation(extent={{82,18},{102,38}})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SinkMdot sink2(
      redeclare package Medium = ThermoCycle.Media.StandardWater,
      Mdot_0=0.1) annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-84,-20})));
    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTpofHotWater( redeclare
        package Medium =
                 ThermoCycle.Media.StandardWater) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={46,32})));
    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTpofColdWater(redeclare
        package Medium =
                 ThermoCycle.Media.StandardWater)
      annotation (Placement(transformation(extent={{-48,-24},{-28,-4}})));
  equation
    connect(source1.flangeB, thermoclineStorage.portHotSF) annotation (Line(
          points={{-49,16},{-28,16},{-28,14.1},{-7.5,14.1}}, color={0,0,255}));

    connect(source2.flangeB, thermoclineStorage.portColdPW) annotation (Line(
          points={{39,0},{24,0},{24,1.65},{7.7,1.65}}, color={0,0,255}));
    connect(thermoclineStorage.portHotPW, sensTpofHotWater.InFlow) annotation (
        Line(points={{7.4,14},{22,14},{22,27.2},{39,27.2}}, color={0,0,255}));
    connect(sensTpofHotWater.OutFlow, sink1.flangeB) annotation (Line(points={{53,
            27.2},{67.5,27.2},{67.5,28},{82.2,28}}, color={0,0,255}));
    connect(thermoclineStorage.portColdSF, sensTpofColdWater.OutFlow) annotation (
       Line(points={{-7.4,1.6},{-18.7,1.6},{-18.7,-18.8},{-31,-18.8}}, color={0,0,
            255}));
    connect(sensTpofColdWater.InFlow, sink2.flangeB) annotation (Line(points={{-45,
            -18.8},{-59.5,-18.8},{-59.5,-20},{-74.2,-20}}, color={0,0,255}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{140,120}})),
      experiment(StopTime=2000),
      __Dymola_experimentSetupOutput,
      Icon(coordinateSystem(extent={{-100,-100},{140,120}})));
  end TankTest;

  package Examples
    extends Modelica.Icons.ExamplesPackage;

    package Simulations
    extends Modelica.Icons.Package;

      package step_by_step
      extends Modelica.Icons.Package;

        package ORC_245fa
        extends Modelica.Icons.Package;

          model step0
          parameter Integer N=10;
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
                package Medium = ThermoCycle.Media.R245fa_CP,  p0=2357000)
              annotation (Placement(transformation(extent={{82,-10},{102,10}})));
           ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
              Mdot_0=0.2588,
              h_0=281455,
              UseT=true,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p=2357000,
              T_0=353.15)
              annotation (Placement(transformation(extent={{-92,-10},{-72,10}})));
           ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim    flow1Dim(
              A=16.18,
              V=0.03781,
              Mdotnom=0.25,
              steadystate=false,
              Mdotconst=true,
              Unom_l=300,
              Unom_tp=700,
              Unom_v=400,
              N=12,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              pstart=2500000,
              Tstart_inlet=323.15,
              Tstart_outlet=323.15)
              annotation (Placement(transformation(extent={{-16,-10},{4,10}})));
          ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot(
              cp=1978,
              rho=928.2,
              Mdot_0=3,
              T_0=418.15)
              annotation (Placement(transformation(extent={{72,56},{52,76}})));
          ThermoCycle.Components.FluidFlow.Pipes.Flow1DConst SecondaryFluid(
              A=16.18,
              V=0.03781,
              steadystate=true,
              Mdotnom=1,
              N=12,
              Unom=100,
              Tstart_inlet=473.15,
              Tstart_outlet=373.15)
              annotation (Placement(transformation(extent={{10,90},{-20,58}})));
           ThermoCycle.Components.HeatFlow.Walls.MetalWall metalWall(
              M_wall=10,
              c_wall=500,
              steadystate_T_wall=true,
              N=12,
              Aext=16,
              Aint=16,
              Tstart_wall_1=423.15,
              Tstart_wall_end=443.15)
              annotation (Placement(transformation(extent={{-28,10},{22,42}})));
          equation
            connect(sourceWF.flangeB, flow1Dim.InFlow) annotation (Line(
                points={{-73,0},{-14.3333,0}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(flow1Dim.OutFlow, sinkPFluid.flangeB) annotation (Line(
                points={{2.33333,0.0833333},{44,0.0833333},{44,0},{83.6,0}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(source_Cdot.flange, SecondaryFluid.flange_Cdot) annotation (Line(
                points={{53.8,65.9},{34.9,65.9},{34.9,74},{10,74}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(SecondaryFluid.Wall_int, metalWall.Wall_int) annotation (Line(
                points={{-5,66},{-3,66},{-3,30.8}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(metalWall.Wall_ext, flow1Dim.Wall_int) annotation (Line(
                points={{-3.5,21.2},{-3.5,12.6},{-6,12.6},{-6,4.16667}},
                color={255,0,0},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                    preserveAspectRatio=true),
                                graphics), Icon(coordinateSystem(extent={{-100,-100},
                      {100,100}})),
              experiment(StopTime=1000),
              __Dymola_experimentSetupOutput);
          end step0;

          model step1

            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
                package Medium = ThermoCycle.Media.R245fa_CP,  p0=2357000)
              annotation (Placement(transformation(extent={{82,-10},{102,10}})));
           ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
              Mdot_0=0.2588,
              h_0=281455,
              UseT=true,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p=2357000,
              T_0=353.15)
              annotation (Placement(transformation(extent={{-92,-10},{-72,10}})));
           ThermoCycle.Components.Units.HeatExchangers.Hx1DConst    hx1DConst(
              steadystate_T_sf=true,
              steadystate_h_wf=true,
              steadystate_T_wall=true,
              N=10,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP)
              annotation (Placement(transformation(extent={{-30,-2},{2,36}})));
          ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot(
              cp=1978,
              rho=928.2,
              Mdot_0=3,
              T_0=418.15)
              annotation (Placement(transformation(extent={{-20,46},{0,66}})));
          equation
            connect(sourceWF.flangeB, hx1DConst.inletWf)
                                                       annotation (Line(
                points={{-73,0},{-66,0},{-66,-2},{-56,-2},{-56,7.5},{-30,7.5}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(hx1DConst.outletWf, sinkPFluid.flangeB)
                                                          annotation (Line(
                points={{2,7.5},{22,7.5},{22,6},{36,6},{36,-2},{83.6,-2},{83.6,0}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(source_Cdot.flange, hx1DConst.inletSf)
                                                         annotation (Line(
                points={{-1.8,55.9},{36,55.9},{36,26.5},{2,26.5}},
                color={255,0,0},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                    preserveAspectRatio=true),
                                graphics), Icon(coordinateSystem(extent={{-100,-100},
                      {100,100}})),
              experiment(StopTime=1000),
              __Dymola_experimentSetupOutput);
          end step1;

          model step2

           ThermoCycle.Components.FluidFlow.Reservoirs.SinkP  sinkPFluid(redeclare
                package Medium = ThermoCycle.Media.R245fa_CP,  p0=2357000)
              annotation (Placement(transformation(extent={{78,-2},{98,18}})));
           ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
              Mdot_0=0.2588,
              h_0=281455,
              UseT=true,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p=2357000,
              T_0=353.15)
              annotation (Placement(transformation(extent={{-92,-10},{-72,10}})));
           ThermoCycle.Components.Units.HeatExchangers.Hx1DConst    hx1DConst(
              N=10,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              steadystate_T_sf=false,
              steadystate_h_wf=false,
              steadystate_T_wall=false,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance)
              annotation (Placement(transformation(extent={{-30,-2},{2,36}})));
          ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot(
              cp=1978,
              rho=928.2,
              Mdot_0=3,
              T_0=418.15)
              annotation (Placement(transformation(extent={{-20,46},{0,66}})));
           ThermoCycle.Components.Units.PdropAndValves.DP dP(
              A=(2*137*77609.9)^(-0.5),
              k=11857.8*137,
              Mdot_nom=0.2588,
              t_init=500,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              constinit=false,
              UseHomotopy=false,
              use_rho_nom=false,
              p_nom=2357000,
              T_nom=413.15,
              DELTAp_lin_nom=3000,
              DELTAp_quad_nom=5150)
              annotation (Placement(transformation(extent={{24,-6},{44,14}})));
          equation
            connect(sourceWF.flangeB, hx1DConst.inletWf)
                                                       annotation (Line(
                points={{-73,0},{-66,0},{-66,-2},{-56,-2},{-56,7.5},{-30,7.5}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(source_Cdot.flange, hx1DConst.inletSf)
                                                         annotation (Line(
                points={{-1.8,55.9},{36,55.9},{36,26.5},{2,26.5}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(hx1DConst.outletWf, dP.InFlow) annotation (Line(
                points={{2,7.5},{14,7.5},{14,4},{25,4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(dP.OutFlow, sinkPFluid.flangeB) annotation (Line(
                points={{43,4},{64,4},{64,8},{79.6,8}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                    preserveAspectRatio=false),
                                graphics), Icon(coordinateSystem(extent={{-100,-100},
                      {100,100}})),
              experiment(StopTime=1000, __Dymola_NumberOfIntervals=5000),
              __Dymola_experimentSetupOutput);
          end step2;

          model step3

           ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
              Mdot_0=0.2588,
              h_0=281455,
              UseT=true,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p=2357000,
              T_0=353.15) annotation (Placement(transformation(extent={{-92,-10},
                      {-72,10}})));
          ThermoCycle.Components.Units.HeatExchangers.Hx1DConst    hx1DConst(
              N=10,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              steadystate_T_sf=false,
              steadystate_h_wf=false,
              steadystate_T_wall=false,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance)
              annotation (Placement(transformation(extent={{-30,-2},{2,36}})));
           ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot(
              cp=1978,
              rho=928.2,
              Mdot_0=3,
              T_0=418.15)
              annotation (Placement(transformation(extent={{-20,46},{0,66}})));
           ThermoCycle.Components.Units.PdropAndValves.DP dP(
              A=(2*137*77609.9)^(-0.5),
              k=11857.8*137,
              Mdot_nom=0.2588,
              t_init=500,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              constinit=false,
              UseHomotopy=false,
              use_rho_nom=false,
              p_nom=2357000,
              T_nom=413.15,
              DELTAp_lin_nom=3000,
              DELTAp_quad_nom=5150)
              annotation (Placement(transformation(extent={{24,-6},{44,14}})));
          ThermoCycle.Components.FluidFlow.Reservoirs.SinkVdot  sinkVdot(
              Vdot_0=1.889e-3,
              h_out=5.04E5,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              pstart=2357000)
              annotation (Placement(transformation(extent={{62,-6},{82,14}})));
          equation
            connect(source_Cdot.flange, hx1DConst.inletSf)
                                                         annotation (Line(
                points={{-1.8,55.9},{36,55.9},{36,26.5},{2,26.5}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(hx1DConst.outletWf, dP.InFlow) annotation (Line(
                points={{2,7.5},{14,7.5},{14,4},{25,4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(dP.OutFlow, sinkVdot.flangeB) annotation (Line(
                points={{43,4},{62.2,4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sourceMdot.flangeB, hx1DConst.inletWf) annotation (Line(
                points={{-73,0},{-62,0},{-62,-4},{-40,-4},{-40,7.5},{-30,7.5}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                    preserveAspectRatio=true),
                                graphics), Icon(coordinateSystem(extent={{-100,-100},
                      {100,100}})),
              experiment(StopTime=1000, __Dymola_NumberOfIntervals=5000),
              __Dymola_experimentSetupOutput);
          end step3;

          model step4

           ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
              Mdot_0=0.2588,
              h_0=281455,
              UseT=true,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p=2357000,
              T_0=353.15)
              annotation (Placement(transformation(extent={{-102,22},{-82,42}})));
           ThermoCycle.Components.Units.HeatExchangers.Hx1DConst hx1DConst(
              N=10,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              steadystate_T_sf=false,
              steadystate_h_wf=false,
              steadystate_T_wall=false,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance)
              annotation (Placement(transformation(extent={{-40,30},{-8,68}})));
          ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot  source_Cdot(
              cp=1978,
              rho=928.2,
              Mdot_0=3,
              T_0=418.15)
              annotation (Placement(transformation(extent={{-30,78},{-10,98}})));
            ThermoCycle.Components.Units.PdropAndValves.DP dP(
              A=(2*137*77609.9)^(-0.5),
              k=11857.8*137,
              Mdot_nom=0.2588,
              t_init=500,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              constinit=false,
              UseHomotopy=false,
              p_nom=2357000,
              T_nom=413.15,
              DELTAp_lin_nom=3000,
              DELTAp_quad_nom=5150,
              use_rho_nom=false)
              annotation (Placement(transformation(extent={{14,26},{34,46}})));
            ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                                    expander(
              ExpType=ThermoCycle.Functions.Enumerations.ExpTypes.ORCNext,
              V_s=1,
              constPinit=false,
              constinit=false,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p_su_start=2357000,
              p_ex_start=153400,
              T_su_start=413.15)
              annotation (Placement(transformation(extent={{40,0},{72,32}})));
            Modelica.Blocks.Sources.Ramp N_rot(
              startTime=50,
              duration=0,
              height=0,
              offset=48.25)  annotation (Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=0,
                  origin={72,64})));
            ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                         generatorNext(Np=1)
              annotation (Placement(transformation(extent={{94,10},{114,30}})));
           ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
                package Medium =
                         ThermoCycle.Media.R245fa_CP,  p0=153400)
              annotation (Placement(transformation(extent={{72,-26},{92,-6}})));
          equation
            connect(sourceWF.flangeB, hx1DConst.inletWf)
                                                       annotation (Line(
                points={{-83,32},{-76,32},{-76,30},{-66,30},{-66,39.5},{-40,39.5}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(source_Cdot.flange, hx1DConst.inletSf)
                                                         annotation (Line(
                points={{-11.8,87.9},{26,87.9},{26,58.5},{-8,58.5}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(hx1DConst.outletWf, dP.InFlow) annotation (Line(
                points={{-8,39.5},{4,39.5},{4,36},{15,36}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(expander.flange_elc,generatorNext. shaft) annotation (Line(
                points={{66.6667,17.3333},{76.4,17.3333},{76.4,20},{95.4,20}},
                color={0,0,0},
                smooth=Smooth.None));
            connect(N_rot.y,generatorNext. f) annotation (Line(
                points={{83,64},{104.4,64},{104.4,29.4}},
                color={0,0,127},
                smooth=Smooth.None));
            connect(dP.OutFlow, expander.InFlow) annotation (Line(
                points={{33,36},{46.9333,36},{46.9333,22.1333}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(expander.OutFlow, sinkPFluid.flangeB) annotation (Line(
                points={{68,8},{68,-5.8},{73.6,-5.8},{73.6,-16}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                    preserveAspectRatio=false),
                                graphics), Icon(coordinateSystem(extent={{-100,-100},
                      {100,100}})),
              experiment(StopTime=1000, __Dymola_NumberOfIntervals=5000),
              __Dymola_experimentSetupOutput);
          end step4;

          model step5

           ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
              Mdot_0=0.2588,
              UseT=false,
              h_0=2.49E5,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p=2357000,
              T_0=353.15)
              annotation (Placement(transformation(extent={{-76,-70},{-56,-50}})));
            ThermoCycle.Components.Units.HeatExchangers.Hx1DConst hx1DConst(
              N=10,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              steadystate_T_sf=false,
              steadystate_h_wf=false,
              steadystate_T_wall=false,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance)
              annotation (Placement(transformation(extent={{-46,28},{-12,60}})));
           ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot(
              cp=1978,
              rho=928.2,
              Mdot_0=3,
              T_0=418.15)
              annotation (Placement(transformation(extent={{-30,78},{-10,98}})));
           ThermoCycle.Components.Units.PdropAndValves.DP dP(
              A=(2*137*77609.9)^(-0.5),
              k=11857.8*137,
              Mdot_nom=0.2588,
              t_init=500,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              constinit=false,
              UseHomotopy=false,
              use_rho_nom=false,
              p_nom=2357000,
              T_nom=413.15,
              DELTAp_lin_nom=3000,
              DELTAp_quad_nom=5150)
              annotation (Placement(transformation(extent={{14,26},{34,46}})));
          ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                                  expander(
              ExpType=ThermoCycle.Functions.Enumerations.ExpTypes.ORCNext,
              V_s=1,
              constPinit=false,
              constinit=false,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p_su_start=2357000,
              p_ex_start=153400,
              T_su_start=413.15)
              annotation (Placement(transformation(extent={{40,0},{72,32}})));
            Modelica.Blocks.Sources.Ramp N_rot(
              startTime=50,
              duration=0,
              height=0,
              offset=48.25)  annotation (Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=0,
                  origin={72,64})));
           ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                        generatorNext(Np=1)
              annotation (Placement(transformation(extent={{94,10},{114,30}})));
           ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
                package Medium =
                         ThermoCycle.Media.R245fa_CP,  p0=153400)
              annotation (Placement(transformation(extent={{54,-76},{74,-56}})));
          ThermoCycle.Components.Units.HeatExchangers.Hx1D    recuperator(
              N=10,
              steadystate_h_cold=true,
              steadystate_h_hot=true,
              steadystate_T_wall=true,
              redeclare model ColdSideHeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
              redeclare model HotSideSideHeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              redeclare package Medium2 = ThermoCycle.Media.R245fa_CP)
              annotation (Placement(transformation(extent={{-20,20},{20,-20}},
                  rotation=90,
                  origin={4,-32})));

          equation
            connect(source_Cdot.flange, hx1DConst.inletSf)
                                                         annotation (Line(
                points={{-11.8,87.9},{26,87.9},{26,52},{-12,52}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(hx1DConst.outletWf, dP.InFlow) annotation (Line(
                points={{-12,36},{15,36}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(expander.flange_elc,generatorNext. shaft) annotation (Line(
                points={{66.6667,17.3333},{76.4,17.3333},{76.4,20},{95.4,20}},
                color={0,0,0},
                smooth=Smooth.None));
            connect(N_rot.y,generatorNext. f) annotation (Line(
                points={{83,64},{104.4,64},{104.4,29.4}},
                color={0,0,127},
                smooth=Smooth.None));
            connect(dP.OutFlow, expander.InFlow) annotation (Line(
                points={{33,36},{46.9333,36},{46.9333,22.1333}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.outlet_fl1, hx1DConst.inletWf) annotation (Line(
                points={{-2.66667,-18.6667},{-2.66667,-2},{-84,-2},{-84,36},{
                    -46,36}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(expander.OutFlow, recuperator.inlet_fl2) annotation (Line(
                points={{68,8},{68,-4},{12,-4},{12,-18.9333}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.outlet_fl2, sinkPFluid.flangeB) annotation (Line(
                points={{11.7333,-45.0667},{11.7333,-66},{55.6,-66}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.inlet_fl1, sourceWF.flangeB) annotation (Line(
                points={{-2.66667,-45.3333},{-2.66667,-60},{-57,-60}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                    preserveAspectRatio=true),
                                graphics), Icon(coordinateSystem(extent={{-100,-100},
                      {100,100}})),
              experiment(StopTime=1000, __Dymola_NumberOfIntervals=5000),
              __Dymola_experimentSetupOutput);
          end step5;

          model step6

           ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
              Mdot_0=0.2588,
              UseT=false,
              h_0=2.49E5,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p=2357000,
              T_0=353.15)
              annotation (Placement(transformation(extent={{-76,-70},{-56,-50}})));
          ThermoCycle.Components.Units.HeatExchangers.Hx1DConst    hx1DConst(
              N=10,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              steadystate_T_sf=false,
              steadystate_h_wf=false,
              steadystate_T_wall=false,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal)
              annotation (Placement(transformation(extent={{-46,28},{-12,60}})));

          ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot(
              cp=1978,
              rho=928.2,
              Mdot_0=3,
              T_0=418.15)
              annotation (Placement(transformation(extent={{-30,78},{-10,98}})));
           ThermoCycle.Components.Units.PdropAndValves.DP dp_hp(
              A=(2*137*77609.9)^(-0.5),
              k=11857.8*137,
              Mdot_nom=0.2588,
              t_init=500,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              constinit=false,
              UseHomotopy=false,
              use_rho_nom=true,
              p_nom=2357000,
              T_nom=413.15,
              DELTAp_lin_nom=3000,
              DELTAp_quad_nom=5150)
              annotation (Placement(transformation(extent={{14,26},{34,46}})));
          ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                                  expander(
              ExpType=ThermoCycle.Functions.Enumerations.ExpTypes.ORCNext,
              V_s=1,
              constPinit=false,
              constinit=false,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p_su_start=2357000,
              p_ex_start=153400,
              T_su_start=413.15)
              annotation (Placement(transformation(extent={{40,0},{72,32}})));
            Modelica.Blocks.Sources.Ramp N_rot(
              startTime=50,
              duration=0,
              height=0,
              offset=48.25)  annotation (Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=0,
                  origin={72,64})));
           ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                        generatorNext(Np=1)
              annotation (Placement(transformation(extent={{94,10},{114,30}})));
          ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
                package Medium =
                         ThermoCycle.Media.R245fa_CP,  p0=153400)
              annotation (Placement(transformation(extent={{54,-76},{74,-56}})));
           ThermoCycle.Components.Units.HeatExchangers.Hx1D       recuperator(
              N=10,
              steadystate_h_cold=true,
              steadystate_h_hot=true,
              steadystate_T_wall=true,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              redeclare model ColdSideHeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              redeclare model HotSideSideHeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              redeclare package Medium2 = ThermoCycle.Media.R245fa_CP)
              annotation (Placement(transformation(extent={{-16,15},{16,-15}},
                  rotation=90,
                  origin={1,-22})));

            ThermoCycle.Components.Units.PdropAndValves.DP dp_lp(
              k=38.4E3*9.5,
              A=(2*9.5*23282.7)^(-0.5),
              Mdot_nom=0.2588,
              use_rho_nom=true,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p_nom=190000,
              T_nom=351.15,
              DELTAp_lin_nom=3000,
              DELTAp_quad_nom=5150)
              annotation (Placement(transformation(extent={{46,-16},{26,4}})));
          equation
            connect(source_Cdot.flange, hx1DConst.inletSf)
                                                         annotation (Line(
                points={{-11.8,87.9},{26,87.9},{26,52},{-12,52}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(hx1DConst.outletWf, dp_hp.InFlow)
                                                   annotation (Line(
                points={{-12,36},{15,36}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(expander.flange_elc,generatorNext. shaft) annotation (Line(
                points={{66.6667,17.3333},{76.4,17.3333},{76.4,20},{95.4,20}},
                color={0,0,0},
                smooth=Smooth.None));
            connect(N_rot.y,generatorNext. f) annotation (Line(
                points={{83,64},{104.4,64},{104.4,29.4}},
                color={0,0,127},
                smooth=Smooth.None));
            connect(dp_hp.OutFlow, expander.InFlow) annotation (Line(
                points={{33,36},{46.9333,36},{46.9333,22.1333}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(expander.OutFlow, dp_lp.InFlow) annotation (Line(
                points={{68,8},{68,-6},{45,-6}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.inlet_fl1, sourceWF.flangeB) annotation (Line(
                points={{-4,-32.6667},{-4,-60},{-57,-60}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.outlet_fl2, sinkPFluid.flangeB) annotation (Line(
                points={{6.8,-32.4533},{6.8,-66},{55.6,-66}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.inlet_fl2, dp_lp.OutFlow) annotation (Line(
                points={{7,-11.5467},{7,-6},{27,-6}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.outlet_fl1, hx1DConst.inletWf) annotation (Line(
                points={{-4,-11.3333},{-4,2},{-66,2},{-66,36},{-46,36}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                    preserveAspectRatio=false),
                                graphics), Icon(coordinateSystem(extent={{-100,-100},
                      {100,100}})),
              experiment(StopTime=1000, __Dymola_NumberOfIntervals=5000),
              __Dymola_experimentSetupOutput);
          end step6;

          model step7

           ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
              Mdot_0=0.2588,
              UseT=false,
              h_0=2.49E5,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p=2357000,
              T_0=353.15)
              annotation (Placement(transformation(extent={{-76,-70},{-56,-50}})));
          ThermoCycle.Components.Units.HeatExchangers.Hx1DConst    hx1DConst(
              N=10,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              steadystate_T_sf=false,
              steadystate_h_wf=false,
              steadystate_T_wall=false,
              redeclare model Medium2HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.IdealFluid.MassFlowDependence,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal)
              annotation (Placement(transformation(extent={{-46,28},{-12,60}})));

          ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot(
              cp=1978,
              rho=928.2,
              Mdot_0=3,
              T_0=418.15)
              annotation (Placement(transformation(extent={{-30,78},{-10,98}})));
            ThermoCycle.Components.Units.PdropAndValves.DP dp_hp(
              A=(2*137*77609.9)^(-0.5),
              k=11857.8*137,
              Mdot_nom=0.2588,
              t_init=500,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              constinit=false,
              UseHomotopy=false,
              p_nom=2357000,
              T_nom=413.15,
              DELTAp_lin_nom=3000,
              DELTAp_quad_nom=5150,
              use_rho_nom=true)
              annotation (Placement(transformation(extent={{14,26},{34,46}})));
          ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                                  expander(
              ExpType=ThermoCycle.Functions.Enumerations.ExpTypes.ORCNext,
              V_s=1,
              constPinit=false,
              constinit=false,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p_su_start=2357000,
              p_ex_start=153400,
              T_su_start=413.15)
              annotation (Placement(transformation(extent={{40,0},{72,32}})));
            Modelica.Blocks.Sources.Ramp N_rot(
              startTime=50,
              duration=0,
              height=0,
              offset=48.25)  annotation (Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=0,
                  origin={72,64})));
           ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                        generatorNext(Np=1)
              annotation (Placement(transformation(extent={{94,10},{114,30}})));
          ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
                package Medium =
                         ThermoCycle.Media.R245fa_CP,  p0=153400)
              annotation (Placement(transformation(extent={{-32,-90},{-52,-70}})));
           ThermoCycle.Components.Units.HeatExchangers.Hx1D       recuperator(
              N=10,
              steadystate_h_cold=true,
              steadystate_T_wall=true,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              steadystate_h_hot=false,
              redeclare model ColdSideHeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              redeclare model HotSideSideHeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              redeclare package Medium2 = ThermoCycle.Media.R245fa_CP)
              annotation (Placement(transformation(extent={{-16,15},{16,-15}},
                  rotation=90,
                  origin={3,-22})));

           ThermoCycle.Components.Units.PdropAndValves.DP dp_lp(redeclare
                package Medium =
                         ThermoCycle.Media.R245fa_CP,
              k=38.4E3*9.5,
              A=(2*9.5*23282.7)^(-0.5),
              Mdot_nom=0.2588,
              p_nom=190000,
              T_nom=351.15,
              DELTAp_lin_nom=3000,
              DELTAp_quad_nom=5150,
              use_rho_nom=true)
              annotation (Placement(transformation(extent={{46,-16},{26,4}})));
           ThermoCycle.Components.Units.HeatExchangers.Hx1DConst    condenser(
              Unom_l=500,
              Unom_tp=1500,
              Unom_v=750,
              Unom_sf=500,
              Mdotnom_sf=4,
              steadystate_T_sf=true,
              steadystate_h_wf=true,
              steadystate_T_wall=false,
              max_der_wf=false,
              filter_dMdt_wf=true,
              N=10,
              redeclare model Medium2HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.IdealFluid.MassFlowDependence,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              pstart_wf=154883,
              Tstart_inlet_wf=316.92,
              Tstart_outlet_wf=298.15,
              Tstart_inlet_sf=293.15,
              Tstart_outlet_sf=296.36)
              annotation (Placement(transformation(extent={{46,-66},{22,-86}})));

          ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot heat_sink(
              cp=4187,
              rho=1000,
              Mdot_0=4,
              T_0=293.15)
              annotation (Placement(transformation(extent={{48,-98},{34,-84}})));
          equation
            connect(source_Cdot.flange, hx1DConst.inletSf)
                                                         annotation (Line(
                points={{-11.8,87.9},{26,87.9},{26,52},{-12,52}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(hx1DConst.outletWf, dp_hp.InFlow)
                                                   annotation (Line(
                points={{-12,36},{15,36}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(expander.flange_elc,generatorNext. shaft) annotation (Line(
                points={{66.6667,17.3333},{76.4,17.3333},{76.4,20},{95.4,20}},
                color={0,0,0},
                smooth=Smooth.None));
            connect(N_rot.y,generatorNext. f) annotation (Line(
                points={{83,64},{104.4,64},{104.4,29.4}},
                color={0,0,127},
                smooth=Smooth.None));
            connect(dp_hp.OutFlow, expander.InFlow) annotation (Line(
                points={{33,36},{46.9333,36},{46.9333,22.1333}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(expander.OutFlow, dp_lp.InFlow) annotation (Line(
                points={{68,8},{68,-6},{45,-6}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(heat_sink.flange,condenser. inletSf) annotation (Line(
                points={{35.26,-91.07},{4,-91.07},{4,-81},{22,-81}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(condenser.outletWf, sinkPFluid.flangeB) annotation (Line(
                points={{22,-71},{0,-71},{0,-72},{-18,-72},{-18,-80},{-33.6,-80}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.inlet_fl2, dp_lp.OutFlow) annotation (Line(
                points={{9,-11.5467},{9,-6},{27,-6}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.outlet_fl1, hx1DConst.inletWf) annotation (Line(
                points={{-2,-11.3333},{-2,2},{-64,2},{-64,36},{-46,36}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.inlet_fl1, sourceWF.flangeB) annotation (Line(
                points={{-2,-32.6667},{-2,-60},{-57,-60}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.outlet_fl2, condenser.inletWf) annotation (Line(
                points={{8.8,-32.4533},{8.8,-46},{54,-46},{54,-71},{46,-71}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                    preserveAspectRatio=false),
                                graphics), Icon(coordinateSystem(extent={{-100,-100},
                      {100,100}})),
              experiment(StopTime=1000, __Dymola_NumberOfIntervals=4000),
              __Dymola_experimentSetupOutput);
          end step7;

          model step8

           ThermoCycle.Components.Units.HeatExchangers.Hx1DConst    hx1DConst(
              N=10,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              steadystate_T_sf=false,
              steadystate_h_wf=false,
              steadystate_T_wall=false,
              Unom_sf=335,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              redeclare model Medium2HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.IdealFluid.MassFlowDependence,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance)
              annotation (Placement(transformation(extent={{-46,28},{-12,60}})));

          ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot(
              cp=1978,
              rho=928.2,
              Mdot_0=3,
              T_0=418.15)
              annotation (Placement(transformation(extent={{-30,78},{-10,98}})));
          ThermoCycle.Components.Units.PdropAndValves.DP  dp_hp(
              A=(2*137*77609.9)^(-0.5),
              k=11857.8*137,
              Mdot_nom=0.2588,
              t_init=500,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              constinit=false,
              UseHomotopy=false,
              use_rho_nom=true,
              p_nom=2357000,
              T_nom=413.15,
              DELTAp_lin_nom=3000,
              DELTAp_quad_nom=5150)
              annotation (Placement(transformation(extent={{14,26},{34,46}})));
          ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                                   expander(
              ExpType=ThermoCycle.Functions.Enumerations.ExpTypes.ORCNext,
              V_s=1,
              constPinit=false,
              constinit=false,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p_su_start=2357000,
              p_ex_start=177800,
              T_su_start=413.15)
              annotation (Placement(transformation(extent={{40,0},{72,32}})));
            Modelica.Blocks.Sources.Ramp N_rot(
              startTime=50,
              duration=0,
              height=0,
              offset=48.25)  annotation (Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=0,
                  origin={72,64})));
           ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                        generatorNext(Np=1)
              annotation (Placement(transformation(extent={{94,10},{114,30}})));
          ThermoCycle.Components.Units.HeatExchangers.Hx1D       recuperator(
              N=10,
              steadystate_h_cold=true,
              steadystate_h_hot=true,
              steadystate_T_wall=true,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              redeclare model ColdSideHeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              redeclare model HotSideSideHeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              redeclare package Medium2 = ThermoCycle.Media.R245fa_CP,
              pstart_hot=177800)
              annotation (Placement(transformation(extent={{-16,15},{16,-15}},
                  rotation=90,
                  origin={1,-22})));

          ThermoCycle.Components.Units.PdropAndValves.DP dp_lp(
              k=38.4E3*9.5,
              A=(2*9.5*23282.7)^(-0.5),
              Mdot_nom=0.2588,
              use_rho_nom=true,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p_nom=190000,
              T_nom=351.15,
              DELTAp_lin_nom=3000,
              DELTAp_quad_nom=5150)
              annotation (Placement(transformation(extent={{46,-14},{26,6}})));
           ThermoCycle.Components.Units.HeatExchangers.Hx1DConst    condenser(
              Unom_l=500,
              Unom_tp=1500,
              Unom_v=750,
              Mdotnom_sf=4,
              steadystate_T_wall=false,
              N=10,
              max_der_wf=true,
              filter_dMdt_wf=false,
              max_drhodt_wf=50,
              steadystate_T_sf=false,
              steadystate_h_wf=true,
              Unom_sf=335,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              redeclare model Medium2HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.IdealFluid.MassFlowDependence,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              pstart_wf=177800,
              Tstart_inlet_wf=316.92,
              Tstart_outlet_wf=298.15,
              Tstart_inlet_sf=293.15,
              Tstart_outlet_sf=296.36)
              annotation (Placement(transformation(extent={{44,-66},{20,-86}})));

           ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot heat_sink(
              cp=4187,
              rho=1000,
              Mdot_0=4,
              T_0=293.15)
              annotation (Placement(transformation(extent={{48,-98},{34,-84}})));
          ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump
                                                                pump(
              PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.ORCNext,
              PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.freq,
              hstart=2.27e5,
              M_dot_start=0.2588,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP)
              annotation (Placement(transformation(extent={{-60,-70},{-36,-46}})));
            Modelica.Blocks.Sources.Ramp f_pp(
              offset=30,
              startTime=50,
              duration=0,
              height=0)      annotation (Placement(transformation(
                  extent={{-6,-6},{6,6}},
                  rotation=0,
                  origin={-74,-26})));
           ThermoCycle.Components.Units.Tanks.Tank_pL tank(
              Vtot=0.015,
              L_start=0.5,
              SteadyState_p=false,
              impose_pressure=true,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              pstart=135000)
              annotation (Placement(transformation(extent={{-28,-92},{-10,-74}})));
          equation
            connect(source_Cdot.flange, hx1DConst.inletSf)
                                                         annotation (Line(
                points={{-11.8,87.9},{26,87.9},{26,52},{-12,52}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(hx1DConst.outletWf, dp_hp.InFlow)
                                                   annotation (Line(
                points={{-12,36},{15,36}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(expander.flange_elc,generatorNext. shaft) annotation (Line(
                points={{66.6667,17.3333},{76.4,17.3333},{76.4,20},{95.4,20}},
                color={0,0,0},
                smooth=Smooth.None));
            connect(N_rot.y,generatorNext. f) annotation (Line(
                points={{83,64},{104.4,64},{104.4,29.4}},
                color={0,0,127},
                smooth=Smooth.None));
            connect(dp_hp.OutFlow, expander.InFlow) annotation (Line(
                points={{33,36},{46.9333,36},{46.9333,22.1333}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(expander.OutFlow, dp_lp.InFlow) annotation (Line(
                points={{68,8},{68,-4},{45,-4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(heat_sink.flange,condenser. inletSf) annotation (Line(
                points={{35.26,-91.07},{4,-91.07},{4,-81},{20,-81}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(condenser.outletWf, tank.InFlow) annotation (Line(
                points={{20,-71},{-20,-71},{-20,-75.44},{-19,-75.44}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(tank.OutFlow, pump.InFlow) annotation (Line(
                points={{-19,-90.92},{-19,-96},{-64,-96},{-64,-57.4},{-56.64,-57.4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(f_pp.y, pump.flow_in) annotation (Line(
                points={{-67.4,-26},{-51.84,-26},{-51.84,-48.4}},
                color={0,0,127},
                smooth=Smooth.None));
            connect(recuperator.inlet_fl2, dp_lp.OutFlow) annotation (Line(
                points={{7,-11.5467},{7,-4},{27,-4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.outlet_fl1, hx1DConst.inletWf) annotation (Line(
                points={{-4,-11.3333},{-4,4},{-68,4},{-68,36},{-46,36}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.outlet_fl2, condenser.inletWf) annotation (Line(
                points={{6.8,-32.4533},{6.8,-44},{62,-44},{62,-71},{44,-71}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.inlet_fl1, pump.OutFlow) annotation (Line(
                points={{-4,-32.6667},{-4,-49.12},{-41.28,-49.12}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                    preserveAspectRatio=false),
                                graphics), Icon(coordinateSystem(extent={{-100,-100},
                      {100,100}})),
              experiment(StopTime=1000, __Dymola_NumberOfIntervals=4000),
              __Dymola_experimentSetupOutput);
          end step8;

          model step9

          ThermoCycle.Components.Units.HeatExchangers.Hx1DConst    hx1DConst(
              N=10,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              steadystate_T_sf=false,
              Unom_sf=335,
              steadystate_h_wf=true,
              steadystate_T_wall=true,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              redeclare model Medium2HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.IdealFluid.MassFlowDependence,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance)
              annotation (Placement(transformation(extent={{-62,46},{-34,70}})));

          ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot(
              cp=1978,
              rho=928.2,
              Mdot_0=3,
              T_0=418.15)
              annotation (Placement(transformation(extent={{-26,72},{-6,92}})));
           ThermoCycle.Components.Units.PdropAndValves.DP dp_hp(
              A=(2*137*77609.9)^(-0.5),
              k=11857.8*137,
              Mdot_nom=0.2588,
              t_init=500,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              constinit=false,
              use_rho_nom=true,
              p_nom=2357000,
              T_nom=413.15,
              DELTAp_lin_nom=3000,
              DELTAp_quad_nom=5150,
              UseHomotopy=false)
              annotation (Placement(transformation(extent={{0,42},{20,62}})));
          ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                                  expander(
              ExpType=ThermoCycle.Functions.Enumerations.ExpTypes.ORCNext,
              V_s=1,
              constPinit=false,
              constinit=false,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p_su_start=2357000,
              p_ex_start=177800,
              T_su_start=413.15)
              annotation (Placement(transformation(extent={{24,20},{56,52}})));
            Modelica.Blocks.Sources.Ramp N_rot(
              startTime=50,
              duration=0,
              height=0,
              offset=48.25)  annotation (Placement(transformation(
                  extent={{-5,-5},{5,5}},
                  rotation=0,
                  origin={65,67})));
          ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                       generatorNext(Np=1)
              annotation (Placement(transformation(extent={{70,20},{98,48}})));
          ThermoCycle.Components.Units.HeatExchangers.Hx1D       recuperator(
              N=10,
              steadystate_h_cold=true,
              steadystate_h_hot=true,
              steadystate_T_wall=true,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              redeclare model ColdSideHeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              redeclare model HotSideSideHeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              redeclare package Medium2 = ThermoCycle.Media.R245fa_CP,
              pstart_hot=177800)
              annotation (Placement(transformation(extent={{-16,15},{16,-15}},
                  rotation=90,
                  origin={-13,-6})));

          ThermoCycle.Components.Units.PdropAndValves.DP  dp_lp(
              k=38.4E3*9.5,
              A=(2*9.5*23282.7)^(-0.5),
              Mdot_nom=0.2588,
              use_rho_nom=true,
              UseHomotopy=false,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              p_nom=190000,
              T_nom=351.15,
              DELTAp_lin_nom=3000,
              DELTAp_quad_nom=5150)
              annotation (Placement(transformation(extent={{32,0},{12,20}})));
          ThermoCycle.Components.Units.HeatExchangers.Hx1DConst    condenser(
              Unom_l=500,
              Unom_tp=1500,
              Unom_v=750,
              Mdotnom_sf=4,
              N=10,
              max_der_wf=true,
              filter_dMdt_wf=false,
              max_drhodt_wf=50,
              steadystate_h_wf=true,
              Unom_sf=335,
              steadystate_T_sf=true,
              steadystate_T_wall=true,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              redeclare model Medium2HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.IdealFluid.MassFlowDependence,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
              redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
              pstart_wf=177800,
              Tstart_inlet_wf=316.92,
              Tstart_outlet_wf=298.15,
              Tstart_inlet_sf=293.15,
              Tstart_outlet_sf=296.36)
              annotation (Placement(transformation(extent={{32,-50},{8,-70}})));

          ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot heat_sink(
              cp=4187,
              rho=1000,
              Mdot_0=4,
              T_0=293.15)
              annotation (Placement(transformation(extent={{34,-82},{20,-68}})));
           ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump
                                                               pump(
              PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.ORCNext,
              PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.freq,
              hstart=2.27e5,
              M_dot_start=0.2588,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP)
              annotation (Placement(transformation(extent={{-74,-54},{-50,-30}})));
            Modelica.Blocks.Sources.Ramp f_pp(
              offset=30,
              startTime=50,
              duration=0,
              height=0)      annotation (Placement(transformation(
                  extent={{-6,-6},{6,6}},
                  rotation=0,
                  origin={-84,-6})));
           ThermoCycle.Components.Units.Tanks.Tank_pL tank(
              Vtot=0.015,
              L_start=0.5,
              SteadyState_p=false,
              impose_pressure=true,
              redeclare package Medium = ThermoCycle.Media.R245fa_CP,
              pstart=135000)
              annotation (Placement(transformation(extent={{-42,-78},{-24,-60}})));
          equation
            connect(source_Cdot.flange, hx1DConst.inletSf)
                                                         annotation (Line(
                points={{-7.8,81.9},{12,81.9},{12,64},{-34,64}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(hx1DConst.outletWf, dp_hp.InFlow)
                                                   annotation (Line(
                points={{-34,52},{1,52}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(expander.flange_elc,generatorNext. shaft) annotation (Line(
                points={{50.6667,37.3333},{62.4,37.3333},{62.4,34},{71.96,34}},
                color={0,0,0},
                smooth=Smooth.None));
            connect(N_rot.y,generatorNext. f) annotation (Line(
                points={{70.5,67},{84.56,67},{84.56,47.16}},
                color={0,0,127},
                smooth=Smooth.None));
            connect(dp_hp.OutFlow, expander.InFlow) annotation (Line(
                points={{19,52},{30.9333,52},{30.9333,42.1333}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(expander.OutFlow, dp_lp.InFlow) annotation (Line(
                points={{52,28},{52,10},{31,10}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(heat_sink.flange,condenser. inletSf) annotation (Line(
                points={{21.26,-75.07},{-10,-75.07},{-10,-65},{8,-65}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(tank.OutFlow, pump.InFlow) annotation (Line(
                points={{-33,-76.92},{-33,-80},{-78,-80},{-78,-41.4},{-70.64,-41.4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(f_pp.y, pump.flow_in) annotation (Line(
                points={{-77.4,-6},{-65.84,-6},{-65.84,-32.4}},
                color={0,0,127},
                smooth=Smooth.None));
            connect(tank.InFlow, condenser.outletWf) annotation (Line(
                points={{-33,-61.44},{-33,-55},{8,-55}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.inlet_fl1, pump.OutFlow) annotation (Line(
                points={{-18,-16.6667},{-18,-33.12},{-55.28,-33.12}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.outlet_fl1, hx1DConst.inletWf) annotation (Line(
                points={{-18,4.66667},{-18,16},{-76,16},{-76,52},{-62,52}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.inlet_fl2, dp_lp.OutFlow) annotation (Line(
                points={{-7,4.45333},{-7,10},{13,10}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(recuperator.outlet_fl2, condenser.inletWf) annotation (Line(
                points={{-7.2,-16.4533},{-7.2,-36},{44,-36},{44,-55},{32,-55}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                    preserveAspectRatio=false),
                                graphics), Icon(coordinateSystem(extent={{-100,-100},
                      {100,100}})),
              experiment(StopTime=1000),
              __Dymola_experimentSetupOutput);
          end step9;
        end ORC_245fa;

        package HeatPump_R407c "5th exercice class: step-by-step resolution"
        extends Modelica.Icons.Package;

          model step1

            ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim flow1Dim(
              Mdotnom=0.044,
              redeclare model Flow1DimHeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              A=4,
              V=0.002,
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              pstart=1650000,
              Tstart_inlet=345.15,
              Tstart_outlet=305.15)
              annotation (Placement(transformation(extent={{18,6},{-18,42}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
              Mdot_0=0.044,
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              p=1600000,
              T_0=345.15)
              annotation (Placement(transformation(extent={{68,14},{48,34}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
                package Medium =
                         ThermoCycle.Media.R407c_CP,
                                                  p0=1650000)
              annotation (Placement(transformation(extent={{-54,14},{-74,34}})));
            ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=10)
              annotation (Placement(transformation(extent={{-10,52},{10,72}})));
            Modelica.Blocks.Sources.Constant const(k=280)
              annotation (Placement(transformation(extent={{-52,72},{-36,88}})));
          equation
            connect(sourceMdot.flangeB, flow1Dim.InFlow) annotation (Line(
                points={{49,24},{15,24}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(flow1Dim.OutFlow, sinkP.flangeB) annotation (Line(
                points={{-15,24.15},{-35.5,24.15},{-35.5,24},{-55.6,24}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(source_T.thermalPort, flow1Dim.Wall_int) annotation (Line(
                points={{-0.1,57.9},{-0.1,52.95},{0,52.95},{0,31.5}},
                color={255,0,0},
                smooth=Smooth.None));
            connect(const.y, source_T.Temperature) annotation (Line(
                points={{-35.2,80},{0,80},{0,66}},
                color={0,0,127},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
                      -100},{100,100}}),      graphics));
          end step1;

          model step2

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
              Mdot_0=0.044,
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              p=1600000,
              T_0=345.15)
              annotation (Placement(transformation(extent={{68,14},{48,34}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
                package Medium =
                         ThermoCycle.Media.R407c_CP,
              h=4E5,
              p0=1650000)
              annotation (Placement(transformation(extent={{-52,14},{-72,34}})));
            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc hx1DInc(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              redeclare package Medium2 = ThermoCycle.Media.Water,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_sf=0.52,
              Mdotnom_wf=0.044,
              A_sf=4,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              pstart_wf=1650000,
              Tstart_inlet_wf=345.15,
              Tstart_outlet_wf=308.15,
              Tstart_inlet_sf=303.15,
              Tstart_outlet_sf=308.15)
              annotation (Placement(transformation(extent={{10,16},{-16,42}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
              redeclare package Medium = ThermoCycle.Media.Water,
              Mdot_0=0.52,
              T_0=298.15)
              annotation (Placement(transformation(extent={{-66,46},{-46,66}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
                package Medium =
                         ThermoCycle.Media.Water)
              annotation (Placement(transformation(extent={{36,44},{56,64}})));
          equation
            connect(sourceMdot.flangeB, hx1DInc.inlet_fl1) annotation (Line(
                points={{49,24},{7,24}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sourceMdot1.flangeB, hx1DInc.inlet_fl2) annotation (Line(
                points={{-47,56},{-32,56},{-32,35},{-12.8,35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(hx1DInc.outlet_fl2, sinkP1.flangeB) annotation (Line(
                points={{6.8,34.8},{24,34.8},{24,54},{37.6,54}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sinkP.flangeB, hx1DInc.outlet_fl1) annotation (Line(
                points={{-53.6,24},{-13,24}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                      -100},{100,100}}),      graphics),
              experiment(StopTime=50),
              __Dymola_experimentSetupOutput);
          end step2;

          model step3

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
              Mdot_0=0.044,
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              p=1600000,
              T_0=345.15)
              annotation (Placement(transformation(extent={{68,14},{48,34}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
                package Medium =
                         ThermoCycle.Media.R407c_CP,
              h=4E5,
              p0=1650000)
              annotation (Placement(transformation(extent={{-50,-42},{-70,-22}})));
            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc hx1DInc(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              redeclare package Medium2 = ThermoCycle.Media.Water,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_sf=0.52,
              Mdotnom_wf=0.044,
              A_sf=4,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              pstart_wf=1650000,
              Tstart_inlet_wf=345.15,
              Tstart_outlet_wf=308.15,
              Tstart_inlet_sf=303.15,
              Tstart_outlet_sf=308.15)
              annotation (Placement(transformation(extent={{10,16},{-16,42}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
              redeclare package Medium = ThermoCycle.Media.Water,
              Mdot_0=0.52,
              T_0=298.15)
              annotation (Placement(transformation(extent={{-66,46},{-46,66}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
                package Medium =
                         ThermoCycle.Media.Water)
              annotation (Placement(transformation(extent={{36,44},{56,64}})));
            ThermoCycle.Components.Units.Tanks.Tank_pL tank_pL(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Vtot=0.004,
              pstart=1650000)
              annotation (Placement(transformation(extent={{-50,-4},{-30,16}})));
          equation
            connect(sourceMdot.flangeB, hx1DInc.inlet_fl1) annotation (Line(
                points={{49,24},{7,24}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sourceMdot1.flangeB, hx1DInc.inlet_fl2) annotation (Line(
                points={{-47,56},{-32,56},{-32,35},{-12.8,35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(hx1DInc.outlet_fl2, sinkP1.flangeB) annotation (Line(
                points={{6.8,34.8},{24,34.8},{24,54},{37.6,54}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(hx1DInc.outlet_fl1, tank_pL.InFlow) annotation (Line(
                points={{-13,24},{-40,24},{-40,14.4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sinkP.flangeB, tank_pL.OutFlow) annotation (Line(
                points={{-51.6,-32},{-40,-32},{-40,-2.8}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
                      -100},{100,100}}),      graphics),
              experiment(StopTime=50),
              __Dymola_experimentSetupOutput);
          end step3;

          model step4

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
              Mdot_0=0.044,
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              p=1600000,
              T_0=345.15)
              annotation (Placement(transformation(extent={{68,14},{48,34}})));
            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc hx1DInc(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              redeclare package Medium2 = ThermoCycle.Media.Water,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_sf=0.52,
              Mdotnom_wf=0.044,
              A_sf=4,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              pstart_wf=1650000,
              Tstart_inlet_wf=345.15,
              Tstart_outlet_wf=308.15,
              Tstart_inlet_sf=303.15,
              Tstart_outlet_sf=308.15)
              annotation (Placement(transformation(extent={{10,16},{-16,42}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
              redeclare package Medium = ThermoCycle.Media.Water,
              Mdot_0=0.52,
              T_0=298.15)
              annotation (Placement(transformation(extent={{-66,46},{-46,66}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
                package Medium =
                         ThermoCycle.Media.Water)
              annotation (Placement(transformation(extent={{36,44},{56,64}})));
            ThermoCycle.Components.Units.Tanks.Tank_pL tank_pL(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Vtot=0.004,
              pstart=1650000)
              annotation (Placement(transformation(extent={{-50,-4},{-30,16}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot2(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Mdot_0=-0.044,
              p=1600000,
              T_0=345.15)
              annotation (Placement(transformation(extent={{-4,-28},{-24,-8}})));
          equation
            connect(sourceMdot.flangeB, hx1DInc.inlet_fl1) annotation (Line(
                points={{49,24},{7,24}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sourceMdot1.flangeB, hx1DInc.inlet_fl2) annotation (Line(
                points={{-47,56},{-32,56},{-32,35},{-12.8,35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(hx1DInc.outlet_fl2, sinkP1.flangeB) annotation (Line(
                points={{6.8,34.8},{24,34.8},{24,54},{37.6,54}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(hx1DInc.outlet_fl1, tank_pL.InFlow) annotation (Line(
                points={{-13,24},{-40,24},{-40,14.4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(tank_pL.OutFlow, sourceMdot2.flangeB) annotation (Line(
                points={{-40,-2.8},{-40,-18},{-23,-18}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                      -100},{100,100}}),      graphics),
              experiment(StopTime=50),
              __Dymola_experimentSetupOutput);
          end step4;

          model step5

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
              Mdot_0=0.044,
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              p=1600000,
              T_0=345.15)
              annotation (Placement(transformation(extent={{68,14},{48,34}})));
            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc hx1DInc(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              redeclare package Medium2 = ThermoCycle.Media.Water,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_sf=0.52,
              Mdotnom_wf=0.044,
              A_sf=4,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              pstart_wf=1650000,
              Tstart_inlet_wf=345.15,
              Tstart_outlet_wf=308.15,
              Tstart_inlet_sf=303.15,
              Tstart_outlet_sf=308.15)
              annotation (Placement(transformation(extent={{10,16},{-16,42}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
              redeclare package Medium = ThermoCycle.Media.Water,
              Mdot_0=0.52,
              T_0=298.15)
              annotation (Placement(transformation(extent={{-66,46},{-46,66}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
                package Medium =
                         ThermoCycle.Media.Water)
              annotation (Placement(transformation(extent={{36,44},{56,64}})));
            ThermoCycle.Components.Units.Tanks.Tank_pL tank_pL(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Vtot=0.004,
              impose_L=true,
              pstart=1650000,
              impose_pressure=true,
              SteadyState_L=false)
              annotation (Placement(transformation(extent={{-50,-4},{-30,16}})));
            ThermoCycle.Components.Units.PdropAndValves.Valve valve(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Mdot_nom=0.044,
              UseNom=false,
              Afull=15e-7,
              Xopen=0.55,
              p_nom=1650000,
              T_nom=308.15,
              DELTAp_nom=1200000,
              use_rho_nom=true)
              annotation (Placement(transformation(extent={{10,-10},{-10,10}},
                  rotation=90,
                  origin={-40,-26})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP2(redeclare
                package Medium =
                         ThermoCycle.Media.R407c_CP,
                                                  p0=380000)
              annotation (Placement(transformation(extent={{-2,-50},{18,-30}})));
          equation
            connect(sourceMdot.flangeB, hx1DInc.inlet_fl1) annotation (Line(
                points={{49,24},{7,24}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sourceMdot1.flangeB, hx1DInc.inlet_fl2) annotation (Line(
                points={{-47,56},{-32,56},{-32,35},{-12.8,35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(hx1DInc.outlet_fl2, sinkP1.flangeB) annotation (Line(
                points={{6.8,34.8},{24,34.8},{24,54},{37.6,54}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(hx1DInc.outlet_fl1, tank_pL.InFlow) annotation (Line(
                points={{-13,24},{-40,24},{-40,14.4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(tank_pL.OutFlow, valve.InFlow) annotation (Line(
                points={{-40,-2.8},{-40,-17}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(valve.OutFlow, sinkP2.flangeB) annotation (Line(
                points={{-40,-35},{-40,-40},{-0.4,-40}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                      -100},{100,100}}),      graphics),
              experiment(StopTime=100),
              __Dymola_experimentSetupOutput);
          end step5;

          model step6

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
              Mdot_0=0.044,
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              p=1600000,
              T_0=345.15)
              annotation (Placement(transformation(extent={{68,14},{48,34}})));
            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc condenser(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              redeclare package Medium2 = ThermoCycle.Media.Water,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_sf=0.52,
              Mdotnom_wf=0.044,
              A_sf=4,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              pstart_wf=1650000,
              Tstart_inlet_wf=345.15,
              Tstart_outlet_wf=308.15,
              Tstart_inlet_sf=303.15,
              Tstart_outlet_sf=308.15)
              annotation (Placement(transformation(extent={{10,16},{-16,42}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
              redeclare package Medium = ThermoCycle.Media.Water,
              Mdot_0=0.52,
              T_0=298.15)
              annotation (Placement(transformation(extent={{-66,46},{-46,66}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
                package Medium =
                         ThermoCycle.Media.Water)
              annotation (Placement(transformation(extent={{36,44},{56,64}})));
            ThermoCycle.Components.Units.Tanks.Tank_pL tank_pL(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Vtot=0.004,
              pstart=1650000)
              annotation (Placement(transformation(extent={{-50,-4},{-30,16}})));
            ThermoCycle.Components.Units.PdropAndValves.Valve valve(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Mdot_nom=0.044,
              UseNom=false,
              Afull=15e-7,
              Xopen=0.55,
              p_nom=1650000,
              T_nom=308.15,
              DELTAp_nom=1200000)
              annotation (Placement(transformation(extent={{10,-10},{-10,10}},
                  rotation=90,
                  origin={-40,-26})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP2(redeclare
                package Medium =
                         ThermoCycle.Media.R407c_CP, p0=380000)
              annotation (Placement(transformation(extent={{42,-62},{62,-42}})));
            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc evaporator(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_wf=0.044,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              redeclare package Medium2 = Modelica.Media.Air.DryAirNasa,
              A_sf=20,
              Unom_sf=100,
              Mdotnom_sf=0.76,
              pstart_wf=460000,
              Tstart_inlet_wf=263.15,
              Tstart_outlet_wf=277.15,
              Tstart_inlet_sf=280.15,
              Tstart_outlet_sf=273.15)
              annotation (Placement(transformation(extent={{-12,-44},{14,-70}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot2(
              redeclare package Medium = Modelica.Media.Air.DryAirNasa,
              Mdot_0=1.76,
              T_0=273.15)
              annotation (Placement(transformation(extent={{48,-90},{28,-70}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP3(redeclare
                package Medium =
                         Modelica.Media.Air.DryAirNasa)
              annotation (Placement(transformation(extent={{-30,-86},{-50,-66}})));
          equation
            connect(sourceMdot.flangeB, condenser.inlet_fl1)
                                                           annotation (Line(
                points={{49,24},{7,24}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sourceMdot1.flangeB, condenser.inlet_fl2)
                                                            annotation (Line(
                points={{-47,56},{-32,56},{-32,35},{-12.8,35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(condenser.outlet_fl2, sinkP1.flangeB)
                                                        annotation (Line(
                points={{6.8,34.8},{24,34.8},{24,54},{37.6,54}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(condenser.outlet_fl1, tank_pL.InFlow)
                                                        annotation (Line(
                points={{-13,24},{-40,24},{-40,14.4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(tank_pL.OutFlow, valve.InFlow) annotation (Line(
                points={{-40,-2.8},{-40,-17}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.outlet_fl1, sinkP2.flangeB) annotation (Line(
                points={{11,-52},{43.6,-52}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.inlet_fl1, valve.OutFlow) annotation (Line(
                points={{-9,-52},{-40,-52},{-40,-35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sourceMdot2.flangeB, evaporator.inlet_fl2) annotation (Line(
                points={{29,-80},{20,-80},{20,-63},{10.8,-63}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.outlet_fl2, sinkP3.flangeB) annotation (Line(
                points={{-8.8,-62.8},{-20,-62.8},{-20,-76},{-31.6,-76}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                      -100},{100,100}}),      graphics),
              experiment(StopTime=100),
              __Dymola_experimentSetupOutput);
          end step6;

          model step7

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
              Mdot_0=0.044,
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              p=1600000,
              T_0=345.15)
              annotation (Placement(transformation(extent={{68,14},{48,34}})));
            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc condenser(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              redeclare package Medium2 = ThermoCycle.Media.StandardWater,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_sf=0.52,
              Mdotnom_wf=0.044,
              A_sf=4,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              pstart_wf=1650000,
              Tstart_inlet_wf=345.15,
              Tstart_outlet_wf=308.15,
              Tstart_inlet_sf=303.15,
              Tstart_outlet_sf=308.15)
              annotation (Placement(transformation(extent={{10,16},{-16,42}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
              redeclare package Medium = ThermoCycle.Media.StandardWater,
              Mdot_0=0.52,
              T_0=298.15)
              annotation (Placement(transformation(extent={{-66,46},{-46,66}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
                package Medium =
                         ThermoCycle.Media.StandardWater)
              annotation (Placement(transformation(extent={{36,44},{56,64}})));
            ThermoCycle.Components.Units.Tanks.Tank_pL tank_pL(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Vtot=0.004,
              pstart=1650000)
              annotation (Placement(transformation(extent={{-50,-4},{-30,16}})));
            ThermoCycle.Components.Units.PdropAndValves.Valve valve(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Mdot_nom=0.044,
              UseNom=false,
              Afull=15e-7,
              Xopen=0.55,
              p_nom=1650000,
              T_nom=308.15,
              DELTAp_nom=1200000)
              annotation (Placement(transformation(extent={{10,-10},{-10,10}},
                  rotation=90,
                  origin={-40,-26})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP2(redeclare
                package Medium =
                         ThermoCycle.Media.R407c_CP,
                                                  p0=1650000)
              annotation (Placement(transformation(extent={{66,-10},{86,10}})));
            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc evaporator(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_wf=0.044,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              redeclare package Medium2 = Modelica.Media.Air.DryAirNasa,
              A_sf=20,
              Unom_sf=100,
              Mdotnom_sf=0.76,
              pstart_wf=460000,
              Tstart_inlet_wf=263.15,
              Tstart_outlet_wf=277.15,
              Tstart_inlet_sf=280.15,
              Tstart_outlet_sf=273.15)
              annotation (Placement(transformation(extent={{-12,-44},{14,-70}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot2(
              redeclare package Medium = Modelica.Media.Air.DryAirNasa,
              Mdot_0=1.76,
              T_0=273.15)
              annotation (Placement(transformation(extent={{48,-90},{28,-70}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP3(redeclare
                package Medium =
                         Modelica.Media.Air.DryAirNasa)
              annotation (Placement(transformation(extent={{-30,-86},{-50,-66}})));
            ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor
                                                                      compressor(
              epsilon_v=0.9,
              V_s=50e-6,
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              p_su_start=380000,
              p_ex_start=1650000,
              T_su_start=278.15) annotation (Placement(transformation(
                  extent={{-12,-12},{12,12}},
                  rotation=180,
                  origin={52,-22})));
            ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                         electricDrive
              annotation (Placement(transformation(extent={{28,-32},{8,-12}})));
            Modelica.Blocks.Sources.Ramp ramp(offset=50)
              annotation (Placement(transformation(extent={{-14,-10},{-4,0}})));
          equation
            connect(sourceMdot.flangeB, condenser.inlet_fl1)
                                                           annotation (Line(
                points={{49,24},{7,24}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sourceMdot1.flangeB, condenser.inlet_fl2)
                                                            annotation (Line(
                points={{-47,56},{-32,56},{-32,35},{-12.8,35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(condenser.outlet_fl2, sinkP1.flangeB)
                                                        annotation (Line(
                points={{6.8,34.8},{24,34.8},{24,54},{37.6,54}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(condenser.outlet_fl1, tank_pL.InFlow)
                                                        annotation (Line(
                points={{-13,24},{-40,24},{-40,14.4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(tank_pL.OutFlow, valve.InFlow) annotation (Line(
                points={{-40,-2.8},{-40,-17}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.inlet_fl1, valve.OutFlow) annotation (Line(
                points={{-9,-52},{-40,-52},{-40,-35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sourceMdot2.flangeB, evaporator.inlet_fl2) annotation (Line(
                points={{29,-80},{20,-80},{20,-63},{10.8,-63}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.outlet_fl2, sinkP3.flangeB) annotation (Line(
                points={{-8.8,-62.8},{-20,-62.8},{-20,-76},{-31.6,-76}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(compressor.OutFlow, sinkP2.flangeB) annotation (Line(
                points={{43.4,-18},{42,-18},{42,0},{67.6,0}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.outlet_fl1, compressor.InFlow) annotation (Line(
                points={{11,-52},{74,-52},{74,-29.8},{58.8,-29.8}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(electricDrive.shaft, compressor.flange_elc) annotation (Line(
                points={{26.6,-22},{44,-22}},
                color={0,0,0},
                smooth=Smooth.None));
            connect(ramp.y, electricDrive.f) annotation (Line(
                points={{-3.5,-5},{17.6,-5},{17.6,-12.6}},
                color={0,0,127},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                      -100},{100,100}}),      graphics),
              experiment(StopTime=100),
              __Dymola_experimentSetupOutput);
          end step7;

          model step8

            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc condenser(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              redeclare package Medium2 = ThermoCycle.Media.StandardWater,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_sf=0.52,
              Mdotnom_wf=0.044,
              A_sf=4,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              pstart_wf=1650000,
              Tstart_inlet_wf=345.15,
              Tstart_outlet_wf=308.15,
              Tstart_inlet_sf=303.15,
              Tstart_outlet_sf=308.15)
              annotation (Placement(transformation(extent={{10,16},{-16,42}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
              redeclare package Medium = ThermoCycle.Media.StandardWater,
              Mdot_0=0.52,
              T_0=298.15)
              annotation (Placement(transformation(extent={{-66,46},{-46,66}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
                package Medium =
                         ThermoCycle.Media.StandardWater)
              annotation (Placement(transformation(extent={{36,44},{56,64}})));
            ThermoCycle.Components.Units.Tanks.Tank_pL tank_pL(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Vtot=0.004,
              pstart=1650000)
              annotation (Placement(transformation(extent={{-50,-4},{-30,16}})));
            ThermoCycle.Components.Units.PdropAndValves.Valve valve(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Mdot_nom=0.044,
              UseNom=false,
              Afull=15e-7,
              Xopen=0.45,
              p_nom=1650000,
              T_nom=308.15,
              DELTAp_nom=1200000)
              annotation (Placement(transformation(extent={{10,-10},{-10,10}},
                  rotation=90,
                  origin={-40,-26})));
            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc evaporator(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_wf=0.044,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              redeclare package Medium2 = Modelica.Media.Air.DryAirNasa,
              A_sf=20,
              Unom_sf=100,
              Mdotnom_sf=0.76,
              pstart_wf=460000,
              Tstart_inlet_wf=263.15,
              Tstart_outlet_wf=277.15,
              Tstart_inlet_sf=280.15,
              Tstart_outlet_sf=273.15)
              annotation (Placement(transformation(extent={{-12,-44},{14,-70}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot2(
              redeclare package Medium = Modelica.Media.Air.DryAirNasa,
              Mdot_0=1.76,
              T_0=273.15)
              annotation (Placement(transformation(extent={{48,-90},{28,-70}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP3(redeclare
                package Medium =
                         Modelica.Media.Air.DryAirNasa)
              annotation (Placement(transformation(extent={{-30,-86},{-50,-66}})));
            ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor
                                                                      compressor(
              epsilon_v=0.9,
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              V_s=85e-6,
              p_su_start=380000,
              p_ex_start=1650000,
              T_su_start=278.15) annotation (Placement(transformation(
                  extent={{-19,-18},{19,18}},
                  rotation=180,
                  origin={59,-16})));
            ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                         electricDrive
              annotation (Placement(transformation(extent={{28,-26},{8,-6}})));
            Modelica.Blocks.Sources.Ramp ramp(offset=50)
              annotation (Placement(transformation(extent={{-14,-2},{-4,8}})));
          equation
            connect(sourceMdot1.flangeB, condenser.inlet_fl2)
                                                            annotation (Line(
                points={{-47,56},{-32,56},{-32,35},{-12.8,35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(condenser.outlet_fl2, sinkP1.flangeB)
                                                        annotation (Line(
                points={{6.8,34.8},{24,34.8},{24,54},{37.6,54}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(condenser.outlet_fl1, tank_pL.InFlow)
                                                        annotation (Line(
                points={{-13,24},{-40,24},{-40,14.4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(tank_pL.OutFlow, valve.InFlow) annotation (Line(
                points={{-40,-2.8},{-40,-17}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.inlet_fl1, valve.OutFlow) annotation (Line(
                points={{-9,-52},{-40,-52},{-40,-35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sourceMdot2.flangeB, evaporator.inlet_fl2) annotation (Line(
                points={{29,-80},{20,-80},{20,-63},{10.8,-63}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.outlet_fl2, sinkP3.flangeB) annotation (Line(
                points={{-8.8,-62.8},{-20,-62.8},{-20,-76},{-31.6,-76}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.outlet_fl1, compressor.InFlow) annotation (Line(
                points={{11,-52},{74,-52},{74,-27.7},{69.7667,-27.7}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(electricDrive.shaft, compressor.flange_elc) annotation (Line(
                points={{26.6,-16},{36.4667,-16},{36.4667,-16},{46.3333,-16}},
                color={0,0,0},
                smooth=Smooth.None));
            connect(ramp.y, electricDrive.f) annotation (Line(
                points={{-3.5,3},{17.6,3},{17.6,-6.6}},
                color={0,0,127},
                smooth=Smooth.None));
            connect(compressor.OutFlow, condenser.inlet_fl1) annotation (Line(
                points={{45.3833,-10},{44,-10},{44,24},{7,24}},
                color={0,0,255},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                      -100},{100,100}}),      graphics),
              experiment(StopTime=100),
              __Dymola_experimentSetupOutput);
          end step8;

          model step9

            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc condenser(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              redeclare package Medium2 = ThermoCycle.Media.StandardWater,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_sf=0.52,
              Mdotnom_wf=0.044,
              A_sf=4,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              steadystate_h_wf=true,
              pstart_wf=1650000,
              Tstart_inlet_wf=345.15,
              Tstart_outlet_wf=308.15,
              Tstart_inlet_sf=303.15,
              Tstart_outlet_sf=303.15)
              annotation (Placement(transformation(extent={{10,16},{-16,42}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
              redeclare package Medium = ThermoCycle.Media.StandardWater,
              Mdot_0=0.52,
              T_0=298.15)
              annotation (Placement(transformation(extent={{-66,46},{-46,66}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
                package Medium =
                         ThermoCycle.Media.StandardWater)
              annotation (Placement(transformation(extent={{36,44},{56,64}})));
            ThermoCycle.Components.Units.Tanks.Tank_pL tank_pL(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Vtot=0.004,
              pstart=1650000)
              annotation (Placement(transformation(extent={{-50,-4},{-30,16}})));
            ThermoCycle.Components.Units.PdropAndValves.Valve valve(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Mdot_nom=0.044,
              UseNom=false,
              Afull=15e-7,
              Xopen=0.45,
              p_nom=1650000,
              T_nom=308.15,
              DELTAp_nom=1200000)
              annotation (Placement(transformation(extent={{10,-10},{-10,10}},
                  rotation=90,
                  origin={-40,-26})));
            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc evaporator(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_wf=0.044,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              redeclare package Medium2 = Modelica.Media.Air.DryAirNasa,
              A_sf=20,
              Unom_sf=100,
              Mdotnom_sf=0.76,
              steadystate_h_wf=true,
              pstart_wf=500000,
              Tstart_inlet_wf=263.15,
              Tstart_outlet_wf=277.15,
              Tstart_inlet_sf=280.15,
              Tstart_outlet_sf=273.15)
              annotation (Placement(transformation(extent={{-12,-44},{14,-70}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot2(
              redeclare package Medium = Modelica.Media.Air.DryAirNasa,
              Mdot_0=1.76,
              T_0=273.15)
              annotation (Placement(transformation(extent={{48,-90},{28,-70}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP3(redeclare
                package Medium =
                         Modelica.Media.Air.DryAirNasa)
              annotation (Placement(transformation(extent={{-30,-86},{-50,-66}})));
            ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor
                                                                      compressor(
              epsilon_v=0.9,
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              V_s=85e-6,
              p_su_start=380000,
              p_ex_start=1650000,
              T_su_start=278.15) annotation (Placement(transformation(
                  extent={{-19,-18},{19,18}},
                  rotation=180,
                  origin={59,-16})));
            ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                         electricDrive
              annotation (Placement(transformation(extent={{28,-26},{8,-6}})));
            ThermoCycle.Components.Units.PdropAndValves.DP dp_ev(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              UseNom=true,
              Mdot_nom=0.044,
              p_nom=380000,
              T_nom=283.15,
              DELTAp_quad_nom=20000)
              annotation (Placement(transformation(extent={{30,-62},{50,-42}})));
            ThermoCycle.Components.Units.PdropAndValves.DP dp_cd(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              UseNom=true,
              Mdot_nom=0.044,
              p_nom=1650000,
              T_nom=345.15,
              DELTAp_quad_nom=20000)
              annotation (Placement(transformation(extent={{38,14},{18,34}})));
            Modelica.Blocks.Sources.Ramp ramp(offset=50,
              height=10,
              duration=2,
              startTime=50)
              annotation (Placement(transformation(extent={{-12,0},{-2,10}})));
          equation
            connect(sourceMdot1.flangeB, condenser.inlet_fl2)
                                                            annotation (Line(
                points={{-47,56},{-32,56},{-32,35},{-12.8,35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(condenser.outlet_fl2, sinkP1.flangeB)
                                                        annotation (Line(
                points={{6.8,34.8},{24,34.8},{24,54},{37.6,54}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(condenser.outlet_fl1, tank_pL.InFlow)
                                                        annotation (Line(
                points={{-13,24},{-40,24},{-40,14.4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(tank_pL.OutFlow, valve.InFlow) annotation (Line(
                points={{-40,-2.8},{-40,-17}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.inlet_fl1, valve.OutFlow) annotation (Line(
                points={{-9,-52},{-40,-52},{-40,-35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sourceMdot2.flangeB, evaporator.inlet_fl2) annotation (Line(
                points={{29,-80},{20,-80},{20,-63},{10.8,-63}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.outlet_fl2, sinkP3.flangeB) annotation (Line(
                points={{-8.8,-62.8},{-20,-62.8},{-20,-76},{-31.6,-76}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(electricDrive.shaft, compressor.flange_elc) annotation (Line(
                points={{26.6,-16},{36.4667,-16},{36.4667,-16},{46.3333,-16}},
                color={0,0,0},
                smooth=Smooth.None));
            connect(evaporator.outlet_fl1, dp_ev.InFlow) annotation (Line(
                points={{11,-52},{31,-52}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(dp_ev.OutFlow, compressor.InFlow) annotation (Line(
                points={{49,-52},{69.7667,-52},{69.7667,-27.7}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(compressor.OutFlow, dp_cd.InFlow) annotation (Line(
                points={{45.3833,-10},{44,-10},{44,24},{37,24}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(dp_cd.OutFlow, condenser.inlet_fl1) annotation (Line(
                points={{19,24},{7,24}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(ramp.y, electricDrive.f) annotation (Line(
                points={{-1.5,5},{17.6,5},{17.6,-6.6}},
                color={0,0,127},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                      -100},{100,100}}),      graphics),
              experiment(StopTime=100),
              __Dymola_experimentSetupOutput);
          end step9;

          model step10

            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc condenser(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              redeclare package Medium2 = ThermoCycle.Media.StandardWater,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_sf=0.52,
              Mdotnom_wf=0.044,
              A_sf=4,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              steadystate_h_wf=true,
              pstart_wf=1650000,
              Tstart_inlet_wf=345.15,
              Tstart_outlet_wf=308.15,
              Tstart_inlet_sf=303.15,
              Tstart_outlet_sf=303.15)
              annotation (Placement(transformation(extent={{10,16},{-16,42}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
              redeclare package Medium = ThermoCycle.Media.StandardWater,
              Mdot_0=0.52,
              T_0=298.15)
              annotation (Placement(transformation(extent={{-66,46},{-46,66}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
                package Medium =
                         ThermoCycle.Media.StandardWater)
              annotation (Placement(transformation(extent={{36,44},{56,64}})));
            ThermoCycle.Components.Units.Tanks.Tank_pL tank_pL(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Vtot=0.004,
              pstart=1650000)
              annotation (Placement(transformation(extent={{-50,-4},{-30,16}})));
            ThermoCycle.Components.Units.PdropAndValves.Valve valve(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              Mdot_nom=0.044,
              UseNom=false,
              Afull=15e-7,
              Xopen=0.45,
              p_nom=1650000,
              T_nom=308.15,
              DELTAp_nom=1200000)
              annotation (Placement(transformation(extent={{10,-10},{-10,10}},
                  rotation=90,
                  origin={-40,-26})));
            ThermoCycle.Components.Units.HeatExchangers.Hx1DInc evaporator(
              redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
              N=10,
              redeclare model Medium1HeatTransferModel =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
              M_wall=10,
              Mdotnom_wf=0.044,
              A_wf=4,
              Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
              V_sf=0.002,
              V_wf=0.002,
              redeclare package Medium2 = Modelica.Media.Air.DryAirNasa,
              A_sf=20,
              Unom_sf=100,
              Mdotnom_sf=0.76,
              steadystate_h_wf=true,
              pstart_wf=500000,
              Tstart_inlet_wf=263.15,
              Tstart_outlet_wf=277.15,
              Tstart_inlet_sf=280.15,
              Tstart_outlet_sf=273.15)
              annotation (Placement(transformation(extent={{-12,-44},{14,-70}})));

            ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot2(
              redeclare package Medium = Modelica.Media.Air.DryAirNasa,
              Mdot_0=1.76,
              T_0=273.15)
              annotation (Placement(transformation(extent={{48,-90},{28,-70}})));
            ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP3(redeclare
                package Medium =
                         Modelica.Media.Air.DryAirNasa)
              annotation (Placement(transformation(extent={{-30,-86},{-50,-66}})));
            ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor
                                                                      compressor(
              epsilon_v=0.9,
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              V_s=85e-6,
              p_su_start=380000,
              p_ex_start=1650000,
              T_su_start=278.15) annotation (Placement(transformation(
                  extent={{-19,-18},{19,18}},
                  rotation=180,
                  origin={59,-16})));
            ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                         electricDrive
              annotation (Placement(transformation(extent={{28,-26},{8,-6}})));
            ThermoCycle.Components.Units.PdropAndValves.DP dp_ev(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              UseNom=true,
              Mdot_nom=0.044,
              p_nom=380000,
              T_nom=283.15,
              DELTAp_quad_nom=20000)
              annotation (Placement(transformation(extent={{30,-62},{50,-42}})));
            ThermoCycle.Components.Units.PdropAndValves.DP dp_cd(
              redeclare package Medium = ThermoCycle.Media.R407c_CP,
              UseNom=true,
              Mdot_nom=0.044,
              p_nom=1650000,
              T_nom=345.15,
              DELTAp_quad_nom=20000)
              annotation (Placement(transformation(extent={{38,14},{18,34}})));
            Modelica.Blocks.Sources.Ramp ramp(offset=50,
              height=10,
              duration=2,
              startTime=50)
              annotation (Placement(transformation(extent={{-12,0},{-2,10}})));
            Modelica.Blocks.Sources.Ramp ramp1(
              height=0.1,
              duration=0,
              offset=0.45,
              startTime=75)
              annotation (Placement(transformation(extent={{-82,-32},{-72,-22}})));
          equation
            connect(sourceMdot1.flangeB, condenser.inlet_fl2)
                                                            annotation (Line(
                points={{-47,56},{-32,56},{-32,35},{-12.8,35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(condenser.outlet_fl2, sinkP1.flangeB)
                                                        annotation (Line(
                points={{6.8,34.8},{24,34.8},{24,54},{37.6,54}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(condenser.outlet_fl1, tank_pL.InFlow)
                                                        annotation (Line(
                points={{-13,24},{-40,24},{-40,14.4}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(tank_pL.OutFlow, valve.InFlow) annotation (Line(
                points={{-40,-2.8},{-40,-17}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.inlet_fl1, valve.OutFlow) annotation (Line(
                points={{-9,-52},{-40,-52},{-40,-35}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(sourceMdot2.flangeB, evaporator.inlet_fl2) annotation (Line(
                points={{29,-80},{20,-80},{20,-63},{10.8,-63}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(evaporator.outlet_fl2, sinkP3.flangeB) annotation (Line(
                points={{-8.8,-62.8},{-20,-62.8},{-20,-76},{-31.6,-76}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(electricDrive.shaft, compressor.flange_elc) annotation (Line(
                points={{26.6,-16},{36.4667,-16},{36.4667,-16},{46.3333,-16}},
                color={0,0,0},
                smooth=Smooth.None));
            connect(evaporator.outlet_fl1, dp_ev.InFlow) annotation (Line(
                points={{11,-52},{31,-52}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(dp_ev.OutFlow, compressor.InFlow) annotation (Line(
                points={{49,-52},{69.7667,-52},{69.7667,-27.7}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(compressor.OutFlow, dp_cd.InFlow) annotation (Line(
                points={{45.3833,-10},{44,-10},{44,24},{37,24}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(dp_cd.OutFlow, condenser.inlet_fl1) annotation (Line(
                points={{19,24},{7,24}},
                color={0,0,255},
                smooth=Smooth.None));
            connect(ramp.y, electricDrive.f) annotation (Line(
                points={{-1.5,5},{17.6,5},{17.6,-6.6}},
                color={0,0,127},
                smooth=Smooth.None));
            connect(ramp1.y, valve.cmd) annotation (Line(
                points={{-71.5,-27},{-60.75,-27},{-60.75,-26},{-48,-26}},
                color={0,0,127},
                smooth=Smooth.None));
            annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                      -100},{100,100}}),      graphics),
              experiment(StopTime=100),
              __Dymola_experimentSetupOutput);
          end step10;
          annotation (Documentation(info="<HTML>
  <p><big> This package allows the user to build a basic heat pump system by following a step-by-step procedure.
  The complete heat pump model is composed by the following components: a compressor (scroll-type), two plate heat exchangers
  one liquid receiver, a valve and two pressure drop model.
  <ul>
  <li><strong><a href=\"modelica://ThermoCycle.Examples.Simulations.step_by_step.HeatPump_R407c.step1\">Step1</a> </strong> We start by modeling the condensation of the working fluid with the following components:
  <ul><li><a href=\"modelica://ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim\">Flow1Dim</a>: It represents the flow of the working fluid
  <li><a href=\"modelica://ThermoCycle.Components.HeatFlow.Sources.Source_T\">Source_T</a>: it represents the temperature source --> it allows the condensation of the fluid
  <li><a href=\"modelica://ThermoCycle.Components.FluidFlow.Reservoirs.SinkP\">SinkP</a>: pressure sink. It imposes the pressure to the system
  <li><a href=\"modelica://ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot\">SourceMdot</a>: Mass flow source. It imposes mass flow and inlet temperature to the system
  </ul>
  <li><strong><a href=\"modelica://ThermoCycle.Examples.Simulations.step_by_step.HeatPump_R407c.step2\">Step2</a> </strong> We replace the Flow1Dim component with an heat exchanger component where the secondary fluid
  is considered incompressible --> <a href=\"modelica://ThermoCycle.Components.Units.HeatExchangers.Hx1DInc\">Hx1DInc</a>. 
  <ul><li>Choose StandardWater as working fluid for the secondary fluid
  <li> Choose upwind-AllowFlowReversal as discretization scheme
  <li> Impose constant heat transfer coefficient in the working fluid side
  <li> Impose an heat transfer coefficient depending on mass flow in the secondary fluid side
  </ul>
<li><strong><a href=\"modelica://ThermoCycle.Examples.Simulations.step_by_step.HeatPump_R407c.step3\">Step3</a> </strong> Add the <a href=\"modelica://ThermoCycle.Components.Units.Tanks.Tank_pL\">Liquid receiver</a> after the condenser. 
The pressure is imposed by the pressure sink connected to the liquid receiver. 

<li><strong><a href=\"modelica://ThermoCycle.Examples.Simulations.step_by_step.HeatPump_R407c.step4\">Step4</a> </strong> Change the pressure sink after the liquid receiver with a volumetric flow sink. In this way
the pressure will be imposed by the tank system.

<li><strong><a href=\"modelica://ThermoCycle.Examples.Simulations.step_by_step.HeatPump_R407c.step5\">Step5</a> </strong> Add the <a href=\"modelica://ThermoCycle.Components.Units.PdropAndValves.Valve\">Valve</a> component after the liquid receiver

<li><strong><a href=\"modelica://ThermoCycle.Examples.Simulations.step_by_step.HeatPump_R407c.step6\">Step6</a> </strong> Add the evaporator after the valve considering the secondary fluid as an incompressible fluid --> <a href=\"modelica://ThermoCycle.Components.Units.HeatExchangers.Hx1DInc\">Hx1DInc</a>.
  <ul><li>Choose Air as working fluid for the secondary fluid
  <li> Choose upwind-AllowFlowReversal as discretization scheme
  <li> Impose constant heat transfer coefficient in the working fluid side
  <li> Impose an heat transfer coefficient depending on mass flow in the secondary fluid side

<li><strong><a href=\"modelica://ThermoCycle.Examples.Simulations.step_by_step.HeatPump_R407c.step7\">Step7</a> </strong> Add the <a href=\"modelica://ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor\">Compressor</a> compoennt and the <a href=\"modelica://ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive\">Electric drive</a> component which
will allow to control the rotational speed fof the compressor. Add finally a constant source from the Modelica library (<a href=\"modelica://Modelica.Blocks.Sources.Constant\">Constant source</a>) to impose a constant rotational speed to the system.

<li><strong><a href=\"modelica://ThermoCycle.Examples.Simulations.step_by_step.HeatPump_R407c.step8\">Step8</a> </strong> Close the cycle and simulate over 100 seconds

<li><strong><a href=\"modelica://ThermoCycle.Examples.Simulations.step_by_step.HeatPump_R407c.step9\">Step9</a> </strong> Add pressure drop that are considered lumped in the lowest vapor density regions of both low and high pressure lines. Simulate over 100 seconds

<li><strong><a href=\"modelica://ThermoCycle.Examples.Simulations.step_by_step.HeatPump_R407c.step10\">Step10</a> </strong> In order to evaluate the dynamic performance of the system impose a variation in the compressor rotational speed at 50s and a variation in the aperture of the valve at 75s.


  </ul>
In order to get a better visualization of the results the authors suggest the use of the ThermoCycle viewer which can be easly downloaded from <a href=\"http://www.thermocycle.net/\">http://www.thermocycle.net/</a>.
  </HTML>"));
        end HeatPump_R407c;
      annotation (Documentation(info="<HTML>
<p><big> This package contains 10 simulations that explain step by step how to build an ORC system model using components from the <b>Thermocycle</b> library
</HTML>"));
      end step_by_step;

      package Plants
      extends Modelica.Icons.Package;

        model ORC_245faInc

        ThermoCycle.Components.Units.HeatExchangers.Hx1DInc     evaporator(
            N=10,
            redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
            Unom_sf=335,
            redeclare package Medium2 =
                ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
            redeclare model Medium1HeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal)
            annotation (Placement(transformation(extent={{-62,44},{-34,68}})));

         ThermoCycle.Components.Units.PdropAndValves.DP dp_hp(
            A=(2*137*77609.9)^(-0.5),
            k=11857.8*137,
            Mdot_nom=0.2588,
            t_init=500,
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            constinit=false,
            use_rho_nom=true,
            UseHomotopy=false,
            p_nom=2357000,
            T_nom=413.15,
            DELTAp_lin_nom=3000,
            DELTAp_quad_nom=5150)
            annotation (Placement(transformation(extent={{0,42},{20,62}})));
         ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                                 expander(
            ExpType=ThermoCycle.Functions.Enumerations.ExpTypes.ORCNext,
            V_s=1,
            constPinit=false,
            constinit=false,
            p_su_start=2357000,
            p_ex_start=177800,
            T_su_start=413.15)
            annotation (Placement(transformation(extent={{24,20},{56,52}})));
          Modelica.Blocks.Sources.Ramp N_rot(
            startTime=50,
            duration=0,
            height=0,
            offset=48.25)  annotation (Placement(transformation(
                extent={{-5,-5},{5,5}},
                rotation=0,
                origin={65,67})));
         ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                  generatorNext(Np=1)
            annotation (Placement(transformation(extent={{70,20},{98,48}})));
         ThermoCycle.Components.Units.HeatExchangers.Hx1D    recuperator(
            N=10,
            steadystate_h_cold=true,
            steadystate_h_hot=true,
            Mdotconst_cold=true,
            Mdotconst_hot=true,
            steadystate_T_wall=true,
            redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
            redeclare package Medium2 = ThermoCycle.Media.R245fa_CP,
            redeclare model ColdSideHeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
            redeclare model HotSideSideHeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
            pstart_hot=177800,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal)
            annotation (Placement(transformation(extent={{-16,15},{16,-15}},
                rotation=90,
                origin={-13,-6})));

        ThermoCycle.Components.Units.PdropAndValves.DP dp_lp(
            k=38.4E3*9.5,
            A=(2*9.5*23282.7)^(-0.5),
            Mdot_nom=0.2588,
            use_rho_nom=true,
            UseHomotopy=false,
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            p_nom=190000,
            T_nom=351.15,
            DELTAp_lin_nom=3000,
            DELTAp_quad_nom=5150)
            annotation (Placement(transformation(extent={{32,0},{12,20}})));
         ThermoCycle.Components.Units.HeatExchangers.Hx1DInc     condenser(
            Unom_l=500,
            Unom_tp=1500,
            Unom_v=750,
            Mdotnom_sf=4,
            steadystate_T_wall=false,
            N=10,
            filter_dMdt_wf=false,
            max_drhodt_wf=50,
            steadystate_h_wf=true,
            Unom_sf=335,
            redeclare package Medium2 =
                Modelica.Media.Water.ConstantPropertyLiquidWater,
            max_der_wf=true,
            redeclare model Medium1HeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
            pstart_wf=177800,
            Tstart_inlet_wf=316.92,
            Tstart_outlet_wf=298.15,
            Tstart_inlet_sf=293.15,
            Tstart_outlet_sf=296.36,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal)
            annotation (Placement(transformation(extent={{30,-50},{6,-70}})));

         ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump
                                                             pump(
            PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.ORCNext,
            PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.freq,
            hstart=2.27e5,
            M_dot_start=0.2588,
            redeclare package Medium = ThermoCycle.Media.R245fa_CP)
            annotation (Placement(transformation(extent={{-74,-54},{-50,-30}})));
          Modelica.Blocks.Sources.Ramp f_pp(
            offset=30,
            startTime=50,
            duration=0,
            height=0)      annotation (Placement(transformation(
                extent={{-6,-6},{6,6}},
                rotation=0,
                origin={-84,-6})));
        ThermoCycle.Components.Units.Tanks.Tank_pL tank(
            Vtot=0.015,
            L_start=0.5,
            SteadyState_p=false,
            impose_pressure=true,
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            pstart=135000)
            annotation (Placement(transformation(extent={{-42,-78},{-24,-60}})));
         ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot source_htf(
            redeclare package Medium =
                ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
            Mdot_0=3,
            T_0=418.15) annotation (Placement(transformation(extent={{2,66},{-18,86}})));
         ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sink_htf(redeclare
              package Medium =
                ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66)
            annotation (Placement(transformation(extent={{-84,68},{-96,80}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot source_cf(
            redeclare package Medium =
                Modelica.Media.Water.ConstantPropertyLiquidWater,
            Mdot_0=4,
            T_0=293.15) annotation (Placement(transformation(extent={{-24,-88},{-10,
                    -74}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sink_htf1(redeclare
              package Medium =
                Modelica.Media.Water.ConstantPropertyLiquidWater, p0=300000)
            annotation (Placement(transformation(extent={{50,-80},{62,-68}})));
        equation
          connect(expander.flange_elc,generatorNext. shaft) annotation (Line(
              points={{50.6667,37.3333},{62.4,37.3333},{62.4,34},{71.96,34}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(N_rot.y,generatorNext. f) annotation (Line(
              points={{70.5,67},{84.56,67},{84.56,47.16}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dp_hp.OutFlow, expander.InFlow) annotation (Line(
              points={{19,52},{30.9333,52},{30.9333,42.1333}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(expander.OutFlow, dp_lp.InFlow) annotation (Line(
              points={{52,28},{52,10},{31,10}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(tank.OutFlow, pump.InFlow) annotation (Line(
              points={{-33,-76.92},{-33,-80},{-78,-80},{-78,-41.4},{-70.64,-41.4}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(f_pp.y, pump.flow_in) annotation (Line(
              points={{-77.4,-6},{-65.84,-6},{-65.84,-32.4}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(recuperator.inlet_fl1, pump.OutFlow) annotation (Line(
              points={{-18,-16.6667},{-18,-33.12},{-55.28,-33.12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.inlet_fl2, dp_lp.OutFlow) annotation (Line(
              points={{-7,4.45333},{-7,10},{13,10}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.outlet_fl2, condenser.inlet_fl1) annotation (Line(
              points={{-7.2,-16.4533},{-7.2,-36},{38,-36},{38,-56.1538},{
                  27.2308,-56.1538}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(condenser.outlet_fl1, tank.InFlow) annotation (Line(
              points={{8.76923,-56.1538},{-14,-56.1538},{-14,-56},{-33,-56},{
                  -33,-61.44}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(source_cf.flangeB, condenser.inlet_fl2) annotation (Line(
              points={{-10.7,-81},{0,-81},{0,-64.6154},{8.95385,-64.6154}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(condenser.outlet_fl2, sink_htf1.flangeB) annotation (Line(
              points={{27.0462,-64.4615},{42,-64.4615},{42,-74},{50.96,-74}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.outlet_fl1, evaporator.inlet_fl1) annotation (Line(
              points={{-18,4.66667},{-18,26},{-84,26},{-84,51.3846},{-58.7692,
                  51.3846}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(evaporator.outlet_fl1, dp_hp.InFlow) annotation (Line(
              points={{-37.2308,51.3846},{-18.1154,51.3846},{-18.1154,52},{1,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(evaporator.inlet_fl2, source_htf.flangeB) annotation (Line(
              points={{-37.4462,61.5385},{-26,61.5385},{-26,76},{-17,76}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(evaporator.outlet_fl2, sink_htf.flangeB) annotation (Line(
              points={{-58.5538,61.3538},{-74,61.3538},{-74,74},{-84.96,74}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                  preserveAspectRatio=true),
                              graphics), Icon(coordinateSystem(extent={{-100,-100},
                    {100,100}})),
            experiment(StopTime=1000),
            __Dymola_experimentSetupOutput,
            Documentation(info="<html>
<p>Dynamic model of an Organic Rankine Cycle with recuperator and R245fa as working fluid.</p>
<p><h4><font color=\"#008000\">Modeling assumptions</font></h4></p>
<p><ul>
<li> The pressure drops are lumped into two punctual pressure drop components in the vapor lines</li>
<li> No control of the cycle is implemented</li>
<li> The properties of R245fa are calculated using CoolProp</li>
<li> The centrifugal pump and the screw expander are modeled using efficiency curves derived from experimental data</li>
</ul></p>
<p><br/>NB: A second version of this model is also provided, in which the properties of R245fa are computed in CoolProp with the TTSE method, which dramatically increases the simulation speed.</p>
</html>"));
        end ORC_245faInc;

        model ORC_245faInc_TTSE
        replaceable package OrganicMedium = ThermoCycle.Media.R245fa_CP_Smooth;
        ThermoCycle.Components.Units.HeatExchangers.Hx1DInc                  evaporator(
            N=10,
            redeclare package Medium1 = OrganicMedium,
            Unom_sf=335,
            redeclare package Medium2 =
                ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
            redeclare model Medium1HeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal)
            annotation (Placement(transformation(extent={{-62,46},{-34,70}})));

         ThermoCycle.Components.Units.PdropAndValves.DP dp_hp(
            A=(2*137*77609.9)^(-0.5),
            k=11857.8*137,
            Mdot_nom=0.2588,
            t_init=500,
            redeclare package Medium = OrganicMedium,
            constinit=false,
            use_rho_nom=true,
            UseHomotopy=false,
            p_nom=2357000,
            T_nom=413.15,
            DELTAp_lin_nom=3000,
            DELTAp_quad_nom=5150)
            annotation (Placement(transformation(extent={{0,42},{20,62}})));
        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                                 expander(
            redeclare package Medium = OrganicMedium,
            ExpType=ThermoCycle.Functions.Enumerations.ExpTypes.ORCNext,
            V_s=1,
            constPinit=false,
            constinit=false,
            p_su_start=2357000,
            p_ex_start=177800,
            T_su_start=413.15)
            annotation (Placement(transformation(extent={{24,20},{56,52}})));
          Modelica.Blocks.Sources.Ramp N_rot(
            startTime=50,
            duration=0,
            height=0,
            offset=48.25)  annotation (Placement(transformation(
                extent={{-5,-5},{5,5}},
                rotation=0,
                origin={65,67})));
          ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                       generatorNext(Np=1)
            annotation (Placement(transformation(extent={{70,20},{98,48}})));
         ThermoCycle.Components.Units.HeatExchangers.Hx1D    recuperator(
            N=10,
            steadystate_h_cold=true,
            steadystate_h_hot=true,
            steadystate_T_wall=true,
            redeclare package Medium1 = OrganicMedium,
            redeclare package Medium2 = OrganicMedium,
            redeclare model ColdSideHeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
            redeclare model HotSideSideHeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
            pstart_hot=177800,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal)
            annotation (Placement(transformation(extent={{-16,15},{16,-15}},
                rotation=90,
                origin={-13,-6})));

        ThermoCycle.Components.Units.PdropAndValves.DP dp_lp(
            k=38.4E3*9.5,
            A=(2*9.5*23282.7)^(-0.5),
            Mdot_nom=0.2588,
            use_rho_nom=true,
            UseHomotopy=false,
            redeclare package Medium = OrganicMedium,
            p_nom=190000,
            T_nom=351.15,
            DELTAp_lin_nom=3000,
            DELTAp_quad_nom=5150)
            annotation (Placement(transformation(extent={{32,0},{12,20}})));
        ThermoCycle.Components.Units.HeatExchangers.Hx1DInc                 condenser(
            Unom_l=500,
            Unom_tp=1500,
            Unom_v=750,
            Mdotnom_sf=4,
            steadystate_T_wall=false,
            N=10,
            max_der_wf=true,
            filter_dMdt_wf=false,
            max_drhodt_wf=50,
            steadystate_h_wf=true,
            Unom_sf=335,
            redeclare package Medium1 =OrganicMedium,
            redeclare package Medium2 =
                Modelica.Media.Water.ConstantPropertyLiquidWater,
            redeclare model Medium1HeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
            pstart_wf=177800,
            Tstart_inlet_wf=316.92,
            Tstart_outlet_wf=298.15,
            Tstart_inlet_sf=293.15,
            Tstart_outlet_sf=296.36,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal)
            annotation (Placement(transformation(extent={{30,-50},{6,-70}})));

         ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump
                                                             pump(
            PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.ORCNext,
            PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.freq,
            hstart=2.27e5,
            M_dot_start=0.2588,
            redeclare package Medium = OrganicMedium)
            annotation (Placement(transformation(extent={{-74,-54},{-50,-30}})));
          Modelica.Blocks.Sources.Ramp f_pp(
            offset=30,
            startTime=50,
            duration=0,
            height=0)      annotation (Placement(transformation(
                extent={{-6,-6},{6,6}},
                rotation=0,
                origin={-84,-6})));
         ThermoCycle.Components.Units.Tanks.Tank_pL tank(
            Vtot=0.015,
            L_start=0.5,
            SteadyState_p=false,
            impose_pressure=true,
            redeclare package Medium = OrganicMedium,
            pstart=135000)
            annotation (Placement(transformation(extent={{-42,-78},{-24,-60}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot source_htf(
            redeclare package Medium =
                ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
            Mdot_0=3,
            T_0=418.15) annotation (Placement(transformation(extent={{2,66},{-18,86}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sink_htf(redeclare
              package Medium =
                ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66)
            annotation (Placement(transformation(extent={{-84,68},{-96,80}})));
         ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot source_cf(
            redeclare package Medium =
                Modelica.Media.Water.ConstantPropertyLiquidWater,
            Mdot_0=4,
            T_0=293.15) annotation (Placement(transformation(extent={{-24,-88},{-10,
                    -74}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sink_htf1(redeclare
              package Medium =
                Modelica.Media.Water.ConstantPropertyLiquidWater, p0=300000)
            annotation (Placement(transformation(extent={{50,-80},{62,-68}})));
        equation
          connect(expander.flange_elc,generatorNext. shaft) annotation (Line(
              points={{50.6667,37.3333},{62.4,37.3333},{62.4,34},{71.96,34}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(N_rot.y,generatorNext. f) annotation (Line(
              points={{70.5,67},{84.56,67},{84.56,47.16}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dp_hp.OutFlow, expander.InFlow) annotation (Line(
              points={{19,52},{30.9333,52},{30.9333,42.1333}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(expander.OutFlow, dp_lp.InFlow) annotation (Line(
              points={{52,28},{52,10},{31,10}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(tank.OutFlow, pump.InFlow) annotation (Line(
              points={{-33,-76.92},{-33,-80},{-78,-80},{-78,-41.4},{-70.64,-41.4}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(f_pp.y, pump.flow_in) annotation (Line(
              points={{-77.4,-6},{-65.84,-6},{-65.84,-32.4}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(recuperator.inlet_fl1, pump.OutFlow) annotation (Line(
              points={{-18,-16.6667},{-18,-33.12},{-55.28,-33.12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.inlet_fl2, dp_lp.OutFlow) annotation (Line(
              points={{-7,4.45333},{-7,10},{13,10}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.outlet_fl2, condenser.inlet_fl1) annotation (Line(
              points={{-7.2,-16.4533},{-7.2,-36},{38,-36},{38,-56.1538},{
                  27.2308,-56.1538}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(condenser.outlet_fl1, tank.InFlow) annotation (Line(
              points={{8.76923,-56.1538},{-14,-56.1538},{-14,-56},{-33,-56},{
                  -33,-61.44}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(source_cf.flangeB, condenser.inlet_fl2) annotation (Line(
              points={{-10.7,-81},{0,-81},{0,-64.6154},{8.95385,-64.6154}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(condenser.outlet_fl2, sink_htf1.flangeB) annotation (Line(
              points={{27.0462,-64.4615},{42,-64.4615},{42,-74},{50.96,-74}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.outlet_fl1, evaporator.inlet_fl1) annotation (Line(
              points={{-18,4.66667},{-18,26},{-84,26},{-84,53.3846},{-58.7692,
                  53.3846}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(evaporator.outlet_fl1, dp_hp.InFlow) annotation (Line(
              points={{-37.2308,53.3846},{-18.1154,53.3846},{-18.1154,52},{1,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(evaporator.inlet_fl2, source_htf.flangeB) annotation (Line(
              points={{-37.4462,63.5385},{-26,63.5385},{-26,76},{-17,76}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(evaporator.outlet_fl2, sink_htf.flangeB) annotation (Line(
              points={{-58.5538,63.3538},{-74,63.3538},{-74,74},{-84.96,74}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                  preserveAspectRatio=true),
                              graphics), Icon(coordinateSystem(extent={{-100,-100},
                    {100,100}})),
            experiment(StopTime=1000),
            __Dymola_experimentSetupOutput);
        end ORC_245faInc_TTSE;

        model ORC_245fa_FMU

         ThermoCycle.Components.Units.HeatExchangers.Hx1DConst hx1DConst(
            N=10,
            redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
            steadystate_T_sf=false,
            steadystate_h_wf=false,
            steadystate_T_wall=false,
            Unom_sf=335,
            redeclare model Medium1HeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal)
            annotation (Placement(transformation(extent={{-92,46},{-64,70}})));

        ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot(
            cp=1978,
            rho=928.2,
            Mdot_0=3,
            T_0=418.15)
            annotation (Placement(transformation(extent={{-94,90},{-74,110}})));
        ThermoCycle.Components.Units.PdropAndValves.DP dp_hp(
            A=(2*137*77609.9)^(-0.5),
            k=11857.8*137,
            Mdot_nom=0.2588,
            t_init=500,
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            constinit=false,
            use_rho_nom=true,
            UseHomotopy=false,
            p_nom=2357000,
            T_nom=413.15,
            DELTAp_lin_nom=3000,
            DELTAp_quad_nom=5150)
            annotation (Placement(transformation(extent={{-22,42},{-2,62}})));
        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                                expander(redeclare
              package Medium =
                ThermoCycle.Media.R245fa_CP,
            ExpType=ThermoCycle.Functions.Enumerations.ExpTypes.ORCNext,
            V_s=1,
            constPinit=false,
            constinit=false,
            p_su_start=2357000,
            p_ex_start=177800,
            T_su_start=413.15)
            annotation (Placement(transformation(extent={{26,20},{58,52}})));
         ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                      generatorNext(Np=1)
            annotation (Placement(transformation(extent={{70,20},{98,48}})));
         ThermoCycle.Components.Units.HeatExchangers.Hx1D    recuperator(redeclare
              package Medium1 =
                ThermoCycle.Media.R245fa_CP,
              redeclare package Medium2 =
                ThermoCycle.Media.R245fa_CP,
            N=10,
            steadystate_h_cold=true,
            steadystate_h_hot=true,
            Mdotconst_cold=true,
            Mdotconst_hot=true,
            steadystate_T_wall=true,
            redeclare model ColdSideHeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
            redeclare model HotSideSideHeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
            pstart_hot=177800)
            annotation (Placement(transformation(extent={{-16,15},{16,-15}},
                rotation=90,
                origin={-13,-6})));

        ThermoCycle.Components.Units.PdropAndValves.DP dp_lp(redeclare package
              Medium =
                ThermoCycle.Media.R245fa_CP,
            k=38.4E3*9.5,
            A=(2*9.5*23282.7)^(-0.5),
            Mdot_nom=0.2588,
            use_rho_nom=true,
            UseHomotopy=false,
            p_nom=190000,
            T_nom=351.15,
            DELTAp_lin_nom=3000,
            DELTAp_quad_nom=5150)
            annotation (Placement(transformation(extent={{32,0},{12,20}})));
        ThermoCycle.Components.Units.HeatExchangers.Hx1DConst condenser(redeclare
              package Medium1 =
                ThermoCycle.Media.R245fa_CP,
            Unom_l=500,
            Unom_tp=1500,
            Unom_v=750,
            Mdotnom_sf=4,
            steadystate_T_wall=false,
            N=10,
            max_der_wf=true,
            filter_dMdt_wf=false,
            max_drhodt_wf=50,
            steadystate_T_sf=false,
            steadystate_h_wf=true,
            Unom_sf=335,
            redeclare model Medium1HeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
            pstart_wf=177800,
            Tstart_inlet_wf=316.92,
            Tstart_outlet_wf=298.15,
            Tstart_inlet_sf=293.15,
            Tstart_outlet_sf=296.36)
            annotation (Placement(transformation(extent={{62,-50},{38,-70}})));

        ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot heat_sink(
            cp=4187,
            rho=1000,
            Mdot_0=4,
            T_0=293.15)
            annotation (Placement(transformation(extent={{68,-98},{50,-80}})));
         ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump
                                                             pump(redeclare
              package Medium =
                ThermoCycle.Media.R245fa_CP,
            PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.ORCNext,
            PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.freq,
            hstart=2.27e5,
            M_dot_start=0.2588)
            annotation (Placement(transformation(extent={{-96,-54},{-72,-30}})));
        ThermoCycle.Components.Units.Tanks.Tank_pL tank(redeclare package
              Medium =
                ThermoCycle.Media.R245fa_CP,
            Vtot=0.015,
            L_start=0.5,
            SteadyState_p=false,
            impose_pressure=true,
            pstart=135000)
            annotation (Placement(transformation(extent={{-42,-78},{-24,-60}})));
         ThermoCycle.Components.FluidFlow.Sensors.SensP sensP(redeclare package
              Medium =
                ThermoCycle.Media.R245fa_CP)
                      annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={-104,-64})));
         ThermoCycle.Components.FluidFlow.Sensors.SensMdot sensMdot(redeclare
              package Medium =
                       ThermoCycle.Media.R245fa_CP)
            annotation (Placement(transformation(extent={{-64,-34},{-44,-14}})));
         ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp( redeclare
              package Medium =
                ThermoCycle.Media.R245fa_CP)
            annotation (Placement(transformation(extent={{10,44},{30,64}})));
         ThermoCycle.Components.HeatFlow.Sensors.SensTsf sensTsf
            annotation (Placement(transformation(extent={{-64,94},{-44,114}})));
          Modelica.Blocks.Interfaces.RealOutput Ths
            annotation (Placement(transformation(extent={{-32,100},{-12,120}})));
          Modelica.Blocks.Interfaces.RealOutput Tev
            annotation (Placement(transformation(extent={{42,60},{62,80}})));
          Modelica.Blocks.Interfaces.RealOutput Pexp
            annotation (Placement(transformation(extent={{-2,64},{-22,84}})));
          Modelica.Blocks.Interfaces.RealOutput Mdot
            annotation (Placement(transformation(extent={{-40,-28},{-20,-8}})));
          Modelica.Blocks.Interfaces.RealOutput Pc annotation (Placement(
                transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={-110,-34})));
          Modelica.Blocks.Interfaces.RealOutput Level
            annotation (Placement(transformation(extent={{-8,-76},{12,-56}})));
          Modelica.Blocks.Interfaces.RealInput Mheat
            annotation (Placement(transformation(extent={{-146,100},{-124,122}})));
          Modelica.Blocks.Interfaces.RealInput Theat
            annotation (Placement(transformation(extent={{-160,76},{-136,100}})));
          Modelica.Blocks.Interfaces.RealInput Mcool
            annotation (Placement(transformation(extent={{136,-86},{110,-60}})));
          Modelica.Blocks.Interfaces.RealInput Tcool
            annotation (Placement(transformation(extent={{140,-116},{116,-92}})));
          Modelica.Blocks.Interfaces.RealInput Up
            annotation (Placement(transformation(extent={{-144,-12},{-118,14}})));
          Modelica.Blocks.Interfaces.RealInput Uexp
            annotation (Placement(transformation(extent={{126,74},{100,100}})));
         ThermoCycle.Components.Units.ControlSystems.Blocks.Tev_SPOptim tev_SPOptim
            annotation (Placement(transformation(extent={{-148,30},{-128,50}})));
        ThermoCycle.Components.Units.ControlSystems.Blocks.DELTAT dELTAT
            annotation (Placement(transformation(extent={{44,96},{64,116}})));
          Modelica.Blocks.Interfaces.RealOutput Tevopt
            annotation (Placement(transformation(extent={{-120,32},{-100,52}})));
          Modelica.Blocks.Interfaces.RealOutput deltaT
            annotation (Placement(transformation(extent={{84,112},{104,132}})));
          Modelica.Blocks.Interfaces.RealOutput Tsat
            annotation (Placement(transformation(extent={{100,104},{120,124}})));
        equation
          connect(hx1DConst.outletWf, dp_hp.InFlow)
                                                 annotation (Line(
              points={{-64,52},{-21,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(expander.flange_elc,generatorNext. shaft) annotation (Line(
              points={{52.6667,37.3333},{62.4,37.3333},{62.4,34},{71.96,34}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(expander.OutFlow, dp_lp.InFlow) annotation (Line(
              points={{54,28},{54,10},{31,10}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(heat_sink.flange,condenser. inletSf) annotation (Line(
              points={{51.62,-89.09},{24,-89.09},{24,-65},{38,-65}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(tank.InFlow, condenser.outletWf) annotation (Line(
              points={{-33,-61.44},{-33,-55},{38,-55}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.outlet_fl1, hx1DConst.inletWf) annotation (Line(
              points={{-18,4.66667},{-18,12},{-98,12},{-98,52},{-92,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.inlet_fl2, dp_lp.OutFlow) annotation (Line(
              points={{-7,4.45333},{-7,10},{13,10}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.outlet_fl2, condenser.inletWf) annotation (Line(
              points={{-7.2,-16.4533},{-7.2,-36},{70,-36},{70,-55},{62,-55}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sensP.OutFlow, pump.InFlow) annotation (Line(
              points={{-100,-58},{-100,-41.4},{-92.64,-41.4}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sensP.InFlow, tank.OutFlow) annotation (Line(
              points={{-100.1,-70},{-56,-70},{-56,-76.92},{-33,-76.92}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(pump.OutFlow, sensMdot.InFlow) annotation (Line(
              points={{-77.28,-33.12},{-66.64,-33.12},{-66.64,-28},{-58,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sensMdot.OutFlow, recuperator.inlet_fl1) annotation (Line(
              points={{-50,-28},{-18,-28},{-18,-16.6667}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(dp_hp.OutFlow, sensTp.InFlow) annotation (Line(
              points={{-3,52},{6,52},{6,49.2},{13,49.2}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sensTp.OutFlow, expander.InFlow) annotation (Line(
              points={{27,49.2},{27,46.6},{32.9333,46.6},{32.9333,42.1333}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(source_Cdot.flange, sensTsf.inlet) annotation (Line(
              points={{-75.8,99.9},{-65.9,99.9},{-65.9,100},{-60,100}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(hx1DConst.inletSf, sensTsf.outlet) annotation (Line(
              points={{-64,64},{-48,64},{-48,78},{-30,78},{-30,100},{-48,100}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(sensTsf.T, Ths) annotation (Line(
              points={{-46,110},{-22,110}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(sensTp.T, Tev) annotation (Line(
              points={{28,60},{38,60},{38,70},{52,70}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(Pexp, sensTp.p) annotation (Line(
              points={{-12,74},{4,74},{4,60},{12,60}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(sensMdot.Mdot, Mdot) annotation (Line(
              points={{-46,-18},{-30,-18}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(sensP.p, Pc) annotation (Line(
              points={{-110,-56},{-110,-34}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(tank.level, Level) annotation (Line(
              points={{-25.62,-65.76},{-13.81,-65.76},{-13.81,-66},{2,-66}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(Up, pump.flow_in) annotation (Line(
              points={{-131,1},{-131,-17.5},{-87.84,-17.5},{-87.84,-32.4}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(Uexp, generatorNext.f) annotation (Line(
              points={{113,87},{84.56,87},{84.56,47.16}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dELTAT.T, sensTp.T) annotation (Line(
              points={{44,101.8},{46,101.8},{46,102},{36,102},{36,60},{28,60}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dELTAT.P, sensTp.p) annotation (Line(
              points={{44.2,112.6},{12,112.6},{12,60}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(tev_SPOptim.p_cd, sensP.p) annotation (Line(
              points={{-148,46.6},{-156,46.6},{-156,-56},{-110,-56}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(tev_SPOptim.T_hf_su, sensTsf.T) annotation (Line(
              points={{-148,41.2},{-150,41.2},{-150,50},{-164,50},{-164,146},{-46,146},
                  {-46,110}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(tev_SPOptim.Mdot, sensMdot.Mdot) annotation (Line(
              points={{-148,35.8},{-170,35.8},{-170,-14},{-46,-14},{-46,-18}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(tev_SPOptim.Tev, Tevopt) annotation (Line(
              points={{-127.4,41.2},{-120.7,41.2},{-120.7,42},{-110,42}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dELTAT.DELTAT, deltaT) annotation (Line(
              points={{64.4,109},{75.2,109},{75.2,122},{94,122}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dELTAT.Tsat, Tsat) annotation (Line(
              points={{64.4,105},{84.2,105},{84.2,114},{110,114}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(Mheat, source_Cdot.M_dot_source) annotation (Line(
              points={{-135,111},{-114,111},{-114,101.8},{-91,101.8}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(Theat, source_Cdot.T_source) annotation (Line(
              points={{-148,88},{-114,88},{-114,97.9},{-91.3,97.9}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(Mcool, heat_sink.M_dot_source) annotation (Line(
              points={{123,-73},{90,-73},{90,-87.38},{65.3,-87.38}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(Tcool, heat_sink.T_source) annotation (Line(
              points={{128,-104},{80,-104},{80,-90.89},{65.57,-90.89}},
              color={0,0,127},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(extent={{-160,-140},{140,140}},
                  preserveAspectRatio=false),
                              graphics), Icon(coordinateSystem(extent={{-160,-140},
                    {140,140}})),
            experiment(StopTime=1000),
            __Dymola_experimentSetupOutput);
        end ORC_245fa_FMU;

        model ORC_DetailedExpander
          "Non-regenerative ORC with double-PID control system and variable Tev, detailed expander model"

        ThermoCycle.Components.Units.Tanks.Tank tank(
            level_start=0.5,
            hstart=2.32e5,
            impose_pressure=true,
            Vtot=0.015,
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            pstart=148400,
            impose_level=true)
            annotation (Placement(transformation(extent={{-44,-32},{-24,-12}})));
         ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump
                                                             Pump(
            X_pp0=0.5539,
            hstart=1.76e5,
            eta_em=0.7,
            M_dot_start=0.3707,
            V_dot_max=6.5e-4,
            PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.FF,
            PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.SQThesis,
            redeclare package Medium = ThermoCycle.Media.R245fa_CP)
            annotation (Placement(transformation(extent={{-66,10},{-86,30}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.Source_Cdot Heat_source(cp=4232)
            annotation (Placement(transformation(extent={{-38,64},{-22,80}})));
         ThermoCycle.Components.Units.HeatExchangers.Hx1DConst Evaporator(
            N=20,
            V_sf=0.003324,
            M_wall=13,
            Mdotnom_sf=0.15,
            Unom_l=1072,
            Unom_tp=3323,
            Unom_v=1359,
            Unom_sf=3855,
            steadystate_T_sf=true,
            V_wf=0.003324,
            A_sf=3.078,
            A_wf=3.078,
            Mdotnom_wf=0.3706,
            max_drhodt_wf=80,
            max_der_wf=true,
            redeclare model Medium1HeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
            redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
            steadystate_h_wf=false,
            pstart_wf=1251000,
            Tstart_inlet_wf=299.96,
            Tstart_outlet_wf=382.55,
            Tstart_inlet_sf=473.15,
            Tstart_outlet_sf=325.41,
            steadystate_T_wall=false)
            annotation (Placement(transformation(extent={{-42,36},{-22,56}})));

        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                     generator(Np=1)
            annotation (Placement(transformation(extent={{82,4},{104,26}})));
        ThermoCycle.Components.Units.HeatExchangers.Hx1DConst Condenser(
            N=20,
            V_sf=0.009562,
            M_wall=30,
            Mdotnom_sf=2,
            Unom_l=425.8,
            Unom_tp=1453,
            Unom_v=477.5,
            Unom_sf=5159,
            V_wf=0.009562,
            A_sf=7.626,
            A_wf=7.626,
            Mdotnom_wf=0.37,
            steadystate_h_wf=true,
            TT_wf=11,
            max_drhodt_wf=40,
            filter_dMdt_wf=true,
            redeclare model Medium1HeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.VaporQualityDependance,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
            redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
            pstart_wf=148400,
            Tstart_inlet_wf=337.91,
            Tstart_outlet_wf=298.25,
            Tstart_inlet_sf=283.15,
            Tstart_outlet_sf=293.25,
            steadystate_T_sf=false,
            steadystate_T_wall=false)
            annotation (Placement(transformation(extent={{6,-12},{-14,8}})));

        ThermoCycle.Components.FluidFlow.Reservoirs.Source_Cdot Heat_sink(rho=1000, cp=4188)
            annotation (Placement(transformation(extent={{-46,-6},{-26,14}})));
        ThermoCycle.Components.FluidFlow.Sensors.SensP sensP(redeclare package
              Medium =
                ThermoCycle.Media.R245fa_CP)
                                          annotation (Placement(transformation(
                extent={{-5,-6},{5,6}},
                rotation=90,
                origin={-80,-11})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp(redeclare package
              Medium =
                ThermoCycle.Media.R245fa_CP)
                                            annotation (Placement(transformation(extent={{16,40},
                    {26,50}})));
        ThermoCycle.Components.FluidFlow.Sensors.SensMdot sensMdot(redeclare
              package Medium =
                       ThermoCycle.Media.R245fa_CP)
            annotation (Placement(transformation(extent={{-70,38},{-58,50}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTsf sensTsf
            annotation (Placement(transformation(extent={{-8,70},{2,78}})));
        ThermoCycle.Components.Units.ControlSystems.SQThesisController
                                                      control_unit1(
            PVmin_freq=-70,
            PVmax_freq=70,
            CSmin_freq=5,
            CSmax_freq=100,
            PVmin_Xpp=0,
            PVmax_Xpp=70,
            CSmin_Xpp=0.01,
            CSstart_Xpp=0.6,
            Kp_freq=-2,
            Kp_Xpp=-1,
            Ti_freq=0.6,
            Ti_Xpp=0.4,
            CSmax_Xpp=1,
            PVstart_Xpp=0.3)
            annotation (Placement(transformation(extent={{36,54},{60,78}})));
        ThermoCycle.Components.FluidFlow.Sources.inputs data
            annotation (Placement(transformation(extent={{-86,66},{-66,78}})));
        ThermoCycle.Components.Units.PdropAndValves.DP DP_cd(
            A=564.1e-6,
            Mdot_nom=0.37,
            UseNom=false,
            use_rho_nom=true,
            UseHomotopy=false,
            constinit=false,
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            DELTAp_0=1000,
            p_nom=177845,
            T_nom=339.15,
            DELTAp_quad_nom=29400)
            annotation (Placement(transformation(extent={{46,-16},{30,0}})));
         ThermoCycle.Components.Units.PdropAndValves.DP DP_ev(
            A=307.9e-6,
            Mdot_nom=0.37,
            use_rho_nom=true,
            UseHomotopy=false,
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            p_nom=1251000,
            T_nom=382.15,
            DELTAp_quad_nom=10814)
            annotation (Placement(transformation(extent={{-16,34},{-2,48}})));
        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ExpanderOpendriveDetailed
                                                                                 expanderOpendriveDetailed(
            HeatCapacity=true,
            V_s=1.1e-4,
            d_su=1.15e-2,
            AU_amb=9.8,
            A_leak=2.6e-6,
            U_dot_w(start=66.7),
            M_dot_n=0.37,
            M_dot_start=0.37,
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            p_su_start=1240000,
            T_su_start=382.55,
            p_su1_start=1100000,
            p_ex_start=178000)
            annotation (Placement(transformation(extent={{50,4},{70,24}})));
        equation
          connect(sensP.p, control_unit1.p_cd) annotation (Line(
              points={{-83.6,-7},{-83.6,-4},{-86,-4},{-86,84},{16,84},{16,73.32},{36.84,
                  73.32}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(sensTsf.T, control_unit1.T_hf_su) annotation (Line(
              points={{1,76.4},{10,76.4},{10,70.44},{36.84,70.44}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(sensMdot.Mdot, control_unit1.Mdot) annotation (Line(
              points={{-59.2,47.6},{-50,47.6},{-50,62},{10,62},{10,67.56},{36.84,67.56}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(sensTp.T, control_unit1.T_su_exp) annotation (Line(
              points={{25,48},{30,48},{30,56},{22,56},{22,64.68},{36.84,64.68}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(sensTp.p, control_unit1.p_su_exp) annotation (Line(
              points={{17,48},{14,48},{14,61.8},{36.84,61.8}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(control_unit1.CS_freq, generator.f) annotation (Line(
              points={{59.28,70.32},{93.44,70.32},{93.44,25.34}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(control_unit1.CS_Xpp, Pump.flow_in) annotation (Line(
              points={{59.28,63.6},{70,63.6},{70,32},{-72.8,32},{-72.8,28}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(data.y[1:2], Heat_source.source) annotation (Line(
              points={{-67.2,71.82},{-50.64,71.82},{-50.64,71.92},{-35.68,71.92}},
              color={0,128,255},
              pattern=LinePattern.Dash,
              smooth=Smooth.None));
          connect(data.y[3:4], Heat_sink.source) annotation (Line(
              points={{-67.2,72.54},{-52,72.54},{-52,3.9},{-43.1,3.9}},
              color={0,128,255},
              pattern=LinePattern.Dash,
              smooth=Smooth.None));
          connect(Heat_sink.flange, Condenser.inletSf) annotation (Line(
              points={{-27.8,3.9},{-20.9,3.9},{-20.9,3},{-14,3}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(Pump.OutFlow, sensMdot.InFlow) annotation (Line(
              points={{-81.6,27.4},{-81.6,41.6},{-66.4,41.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sensMdot.OutFlow, Evaporator.inletWf) annotation (Line(
              points={{-61.6,41.6},{-51.8,41.6},{-51.8,41},{-42,41}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Evaporator.outletWf, DP_ev.InFlow) annotation (Line(
              points={{-22,41},{-15.3,41}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(DP_ev.OutFlow, sensTp.InFlow) annotation (Line(
              points={{-2.7,41},{7.65,41},{7.65,42.6},{17.5,42.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(DP_cd.OutFlow, Condenser.inletWf) annotation (Line(
              points={{30.8,-8},{18,-8},{18,-7},{6,-7}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Condenser.outletWf, tank.InFlow) annotation (Line(
              points={{-14,-7},{-24,-7},{-24,-8},{-34,-8},{-34,-13.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(tank.OutFlow, sensP.InFlow) annotation (Line(
              points={{-34,-30.8},{-34,-38},{-77.66,-38},{-77.66,-14}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sensP.OutFlow, Pump.InFlow) annotation (Line(
              points={{-77.6,-8},{-78,-8},{-78,0},{-58,0},{-58,20.5},{-68.8,20.5}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Heat_source.flange, sensTsf.inlet) annotation (Line(
              points={{-23.44,71.92},{-14.72,71.92},{-14.72,72.4},{-6,72.4}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(sensTsf.outlet, Evaporator.inletSf) annotation (Line(
              points={{0,72.4},{4,72.4},{4,72},{6,72},{6,51},{-22,51}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(expanderOpendriveDetailed.flange_elc, generator.shaft) annotation (
              Line(
              points={{68,15},{83.54,15}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(expanderOpendriveDetailed.flangeB, DP_cd.InFlow) annotation (Line(
              points={{68.6,8},{70,8},{70,-8},{45.2,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sensTp.OutFlow, expanderOpendriveDetailed.flangeA) annotation (Line(
              points={{24.5,42.6},{53.4,42.6},{53.4,18.8}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                    -100},{100,100}}),
                              graphics),
            experiment(StopTime=1669),
            __Dymola_experimentSetupOutput(equdistant=false));
        end ORC_DetailedExpander;

        model SQThesisModel
          "Non-regenerative ORC with double-PID control system and variable Tev"

         ThermoCycle.Components.Units.Tanks.Tank tank(
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            level_start=0.5,
            hstart=2.32e5,
            impose_pressure=true,
            Vtot=0.015,
            pstart=148400,
            impose_level=false)
            annotation (Placement(transformation(extent={{-44,-32},{-24,-12}})));
         ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump
                                                             Pump(
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            X_pp0=0.5539,
            hstart=1.76e5,
            eta_em=0.7,
            M_dot_start=0.3707,
            V_dot_max=6.5e-4,
            PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.FF,
            PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.SQThesis)
            annotation (Placement(transformation(extent={{-66,10},{-86,30}})));
          ThermoCycle.Components.FluidFlow.Reservoirs.Source_Cdot Heat_source(cp=4232)
            annotation (Placement(transformation(extent={{-38,64},{-22,80}})));
          ThermoCycle.Obsolete.Hx_06122013 Evaporator(
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            N=20,
            V_sf=0.003324,
            M_wall=13,
            Mdotnom_sf=0.15,
            Unom_l=1072,
            Unom_tp=3323,
            Unom_v=1359,
            Unom_sf=3855,
            V_wf=0.003324,
            A=3.078,
            Mdotnom_wf=0.3706,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.centr_diff,
            steadystate_T_wall=false,
            steadystate_T_sf=true,
            steadystate_h_wf=false,
            Mdotconst_wf=true,
            pstart_wf=1251000,
            Tstart_inlet_wf=299.96,
            Tstart_outlet_wf=382.55,
            Tstart_inlet_sf=473.15,
            Tstart_outlet_sf=325.41)
            annotation (Placement(transformation(extent={{-42,36},{-22,56}})));
        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                     generator(Np=1)
            annotation (Placement(transformation(extent={{82,4},{104,26}})));
        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                                expander(
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            epsilon_s=0.69,
            FF_exp=1.016,
            V_s=1.1e-4,
            constPinit=false,
            epsilon_start=0.69,
            FF_start=1.016,
            constinit=true,
            t_init=11,
            ExpType=ThermoCycle.Functions.Enumerations.ExpTypes.ODExp,
            p_su_start=1240000,
            p_ex_start=178000,
            T_su_start=382.55)
            annotation (Placement(transformation(extent={{54,2},{78,26}})));
        ThermoCycle.Obsolete.Hx_06122013  Condenser(
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            N=20,
            V_sf=0.009562,
            M_wall=30,
            Mdotnom_sf=2,
            Unom_l=425.8,
            Unom_tp=1453,
            Unom_v=477.5,
            Unom_sf=5159,
            V_wf=0.009562,
            A=7.626,
            Mdotnom_wf=0.37,
            Mdotconst_wf=true,
            max_der_wf=false,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.centr_diff,
            steadystate_T_wall=false,
            steadystate_T_sf=false,
            steadystate_h_wf=true,
            pstart_wf=148400,
            Tstart_inlet_wf=337.91,
            Tstart_outlet_wf=298.25,
            Tstart_inlet_sf=283.15,
            Tstart_outlet_sf=293.25)
            annotation (Placement(transformation(extent={{6,-12},{-14,8}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.Source_Cdot  Heat_sink(rho=1000, cp=4188)
            annotation (Placement(transformation(extent={{-46,-6},{-26,14}})));
        ThermoCycle.Components.FluidFlow.Sensors.SensP sensP(redeclare package
              Medium =
                ThermoCycle.Media.R245fa_CP)
                                          annotation (Placement(transformation(
                extent={{-5,-6},{5,6}},
                rotation=90,
                origin={-80,-11})));
         ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp(redeclare
              package Medium =
                ThermoCycle.Media.R245fa_CP)
                                            annotation (Placement(transformation(extent={{16,40},
                    {26,50}})));
         ThermoCycle.Components.FluidFlow.Sensors.SensMdot sensMdot(redeclare
              package Medium =
                ThermoCycle.Media.R245fa_CP)
            annotation (Placement(transformation(extent={{-70,38},{-58,50}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTsf sensTsf
            annotation (Placement(transformation(extent={{-8,70},{2,78}})));
        ThermoCycle.Components.Units.ControlSystems.SQThesisController    control_unit1(
            PVmin_freq=-70,
            PVmax_freq=70,
            CSmin_freq=5,
            CSmax_freq=100,
            PVmin_Xpp=0,
            PVmax_Xpp=70,
            CSmin_Xpp=0.01,
            CSstart_Xpp=0.6,
            Kp_freq=-2,
            Kp_Xpp=-1,
            Ti_freq=0.6,
            Ti_Xpp=0.4,
            CSmax_Xpp=1,
            PVstart_Xpp=0.3)
            annotation (Placement(transformation(extent={{36,54},{60,78}})));
        ThermoCycle.Components.FluidFlow.Sources.inputs data
            annotation (Placement(transformation(extent={{-86,66},{-66,78}})));
         ThermoCycle.Components.Units.PdropAndValves.DP DP_cd(
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            A=564.1e-6,
            Mdot_nom=0.37,
            UseHomotopy=true,
            constinit=false,
            UseNom=false,
            use_rho_nom=true,
            DELTAp_0=1000,
            p_nom=177845,
            T_nom=339.15,
            DELTAp_quad_nom=29400)
            annotation (Placement(transformation(extent={{44,-18},{28,-2}})));
        ThermoCycle.Components.Units.PdropAndValves.DP DP_ev(
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            A=307.9e-6,
            Mdot_nom=0.37,
            UseHomotopy=false,
            constinit=true,
            use_rho_nom=true,
            p_nom=1251000,
            T_nom=382.15,
            DELTAp_quad_nom=10814)
            annotation (Placement(transformation(extent={{-16,34},{-2,48}})));
        equation
          connect(expander.flange_elc,generator. shaft) annotation (Line(
              points={{74,15},{83.54,15}},
              color={0,0,0},
              smooth=Smooth.None,
              thickness=0.5));
          connect(sensP.p, control_unit1.p_cd) annotation (Line(
              points={{-83.6,-7},{-83.6,6},{-88,6},{-88,84},{16,84},{16,73.32},{36.84,73.32}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(sensTsf.T, control_unit1.T_hf_su) annotation (Line(
              points={{1,76.4},{10,76.4},{10,70.44},{36.84,70.44}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(sensMdot.Mdot, control_unit1.Mdot) annotation (Line(
              points={{-59.2,47.6},{-50,47.6},{-50,62},{10,62},{10,67.56},{36.84,67.56}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(sensTp.T, control_unit1.T_su_exp) annotation (Line(
              points={{25,48},{30,48},{30,56},{22,56},{22,64.68},{36.84,64.68}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(sensTp.p, control_unit1.p_su_exp) annotation (Line(
              points={{17,48},{14,48},{14,61.8},{36.84,61.8}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(control_unit1.CS_freq, generator.f) annotation (Line(
              points={{59.28,70.32},{93.44,70.32},{93.44,25.34}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(control_unit1.CS_Xpp, Pump.flow_in) annotation (Line(
              points={{59.28,63.6},{70,63.6},{70,32},{-72.8,32},{-72.8,28}},
              color={0,0,127},
              pattern=LinePattern.Dot,
              smooth=Smooth.None));
          connect(data.y[1:2], Heat_source.source) annotation (Line(
              points={{-67.2,71.82},{-50.64,71.82},{-50.64,71.92},{-35.68,71.92}},
              color={0,128,255},
              pattern=LinePattern.Dash,
              smooth=Smooth.None));
          connect(data.y[3:4], Heat_sink.source) annotation (Line(
              points={{-67.2,72.54},{-52,72.54},{-52,3.9},{-43.1,3.9}},
              color={0,128,255},
              pattern=LinePattern.Dash,
              smooth=Smooth.None));
          connect(Heat_sink.flange, Condenser.inletSf) annotation (Line(
              points={{-27.8,3.9},{-20.9,3.9},{-20.9,3},{-14,3}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(Pump.OutFlow, sensMdot.InFlow) annotation (Line(
              points={{-81.6,27.4},{-81.6,41.6},{-66.4,41.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sensMdot.OutFlow, Evaporator.inletWf) annotation (Line(
              points={{-61.6,41.6},{-51.8,41.6},{-51.8,41},{-42,41}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Evaporator.outletWf, DP_ev.InFlow) annotation (Line(
              points={{-22,41},{-15.3,41}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(DP_ev.OutFlow, sensTp.InFlow) annotation (Line(
              points={{-2.7,41},{7.65,41},{7.65,42.6},{17.5,42.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sensTp.OutFlow, expander.InFlow) annotation (Line(
              points={{24.5,42.6},{59.2,42.6},{59.2,18.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(expander.OutFlow, DP_cd.InFlow) annotation (Line(
              points={{75,8},{75,-10},{43.2,-10}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(DP_cd.OutFlow, Condenser.inletWf) annotation (Line(
              points={{28.8,-10},{18,-10},{18,-7},{6,-7}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Condenser.outletWf, tank.InFlow) annotation (Line(
              points={{-14,-7},{-24,-7},{-24,-8},{-34,-8},{-34,-13.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(tank.OutFlow, sensP.InFlow) annotation (Line(
              points={{-34,-30.8},{-34,-38},{-77.66,-38},{-77.66,-14}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sensP.OutFlow, Pump.InFlow) annotation (Line(
              points={{-77.6,-8},{-78,-8},{-78,0},{-58,0},{-58,20.5},{-68.8,20.5}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Heat_source.flange, sensTsf.inlet) annotation (Line(
              points={{-23.44,71.92},{-14.72,71.92},{-14.72,72.4},{-6,72.4}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(sensTsf.outlet, Evaporator.inletSf) annotation (Line(
              points={{0,72.4},{4,72.4},{4,72},{6,72},{6,51},{-22,51}},
              color={255,0,0},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                    -100},{100,100}}),
                              graphics),
            experiment(StopTime=1669),
            __Dymola_experimentSetupOutput(equdistant=false),
            Documentation(info="<html>
<p>Dynamic model of a non-recuperative Organic Rankine Cycle with its control system.</p>
<p><h4><font color=\"#008000\">Modeling assumptions</font></h4></p>
<p><ul>
<li>The pressure drops are lumped into two punctual pressure drop components in the vapor lines</li>
<li>The control is performed using two PI controllers</li>
<li>The superheating is controlled with the pump speed and the evaporation temperature is controlled with the expander speed.</li>
<li>The properties of R245fa are calculated using CoolProp</li>
<li>The volumetric pump and the scroll expander are modeled using efficiency curves derived from experimental data</li>
</ul></p>
<p><h4><font color=\"#008000\">Reference</font></h4></p>
<p>Quoilin, S. (2011). <a href=\"http://hdl.handle.net/2268/96436\">Sustainable energy conversion through the use of Organic Rankine Cycles for waste heat recovery and solar applications</a>. Doctoral thesis, University of Li&egrave;ge, ​Li&egrave;ge, ​​Belgium.</p>
</html>"));
        end SQThesisModel;

        model HeatPump_R407c

          ThermoCycle.Components.Units.HeatExchangers.Hx1DInc condenser(
            redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
            redeclare package Medium2 = ThermoCycle.Media.StandardWater,
            N=10,
            redeclare model Medium1HeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
            M_wall=10,
            Mdotnom_sf=0.52,
            Mdotnom_wf=0.044,
            A_sf=4,
            A_wf=4,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
            V_sf=0.002,
            V_wf=0.002,
            steadystate_h_wf=true,
            pstart_wf=1650000,
            Tstart_inlet_wf=345.15,
            Tstart_outlet_wf=308.15,
            Tstart_inlet_sf=303.15,
            Tstart_outlet_sf=303.15)
            annotation (Placement(transformation(extent={{10,16},{-16,42}})));

          ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
            redeclare package Medium = ThermoCycle.Media.StandardWater,
            Mdot_0=0.52,
            T_0=298.15)
            annotation (Placement(transformation(extent={{-66,46},{-46,66}})));
          ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
              package Medium =
                       ThermoCycle.Media.StandardWater)
            annotation (Placement(transformation(extent={{36,44},{56,64}})));
          ThermoCycle.Components.Units.Tanks.Tank_pL tank_pL(
            redeclare package Medium = ThermoCycle.Media.R407c_CP,
            Vtot=0.004,
            pstart=1650000)
            annotation (Placement(transformation(extent={{-50,-4},{-30,16}})));
          ThermoCycle.Components.Units.PdropAndValves.Valve valve(
            redeclare package Medium = ThermoCycle.Media.R407c_CP,
            Mdot_nom=0.044,
            UseNom=false,
            Afull=15e-7,
            Xopen=0.45,
            p_nom=1650000,
            T_nom=308.15,
            DELTAp_nom=1200000)
            annotation (Placement(transformation(extent={{10,-10},{-10,10}},
                rotation=90,
                origin={-40,-26})));
          ThermoCycle.Components.Units.HeatExchangers.Hx1DInc evaporator(
            redeclare package Medium1 = ThermoCycle.Media.R407c_CP,
            N=10,
            redeclare model Medium1HeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
            M_wall=10,
            Mdotnom_wf=0.044,
            A_wf=4,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
            V_sf=0.002,
            V_wf=0.002,
            redeclare package Medium2 = Modelica.Media.Air.DryAirNasa,
            A_sf=20,
            Unom_sf=100,
            Mdotnom_sf=0.76,
            steadystate_h_wf=true,
            pstart_wf=500000,
            Tstart_inlet_wf=263.15,
            Tstart_outlet_wf=277.15,
            Tstart_inlet_sf=280.15,
            Tstart_outlet_sf=273.15)
            annotation (Placement(transformation(extent={{-12,-44},{14,-70}})));

          ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot2(
            redeclare package Medium = Modelica.Media.Air.DryAirNasa,
            Mdot_0=1.76,
            T_0=273.15)
            annotation (Placement(transformation(extent={{48,-90},{28,-70}})));
          ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP3(redeclare
              package Medium =
                       Modelica.Media.Air.DryAirNasa)
            annotation (Placement(transformation(extent={{-30,-86},{-50,-66}})));
          ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor
                                                                    compressor(
            epsilon_v=0.9,
            redeclare package Medium = ThermoCycle.Media.R407c_CP,
            V_s=85e-6,
            p_su_start=380000,
            p_ex_start=1650000,
            T_su_start=278.15) annotation (Placement(transformation(
                extent={{-19,-18},{19,18}},
                rotation=180,
                origin={59,-16})));
          ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                       electricDrive
            annotation (Placement(transformation(extent={{28,-26},{8,-6}})));
          ThermoCycle.Components.Units.PdropAndValves.DP dp_ev(
            redeclare package Medium = ThermoCycle.Media.R407c_CP,
            UseNom=true,
            Mdot_nom=0.044,
            p_nom=380000,
            T_nom=283.15,
            DELTAp_quad_nom=20000)
            annotation (Placement(transformation(extent={{18,-62},{38,-42}})));
          ThermoCycle.Components.Units.PdropAndValves.DP dp_cd(
            redeclare package Medium = ThermoCycle.Media.R407c_CP,
            UseNom=true,
            Mdot_nom=0.044,
            p_nom=1650000,
            T_nom=345.15,
            DELTAp_quad_nom=20000)
            annotation (Placement(transformation(extent={{38,14},{18,34}})));
          Modelica.Blocks.Sources.Ramp ramp(offset=50,
            height=10,
            duration=2,
            startTime=50)
            annotation (Placement(transformation(extent={{-12,0},{-2,10}})));
          ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp(redeclare
              package Medium = ThermoCycle.Media.R407c_CP)
            annotation (Placement(transformation(extent={{46,-56},{62,-40}})));
          ThermoCycle.Components.Units.ControlSystems.SH_block sH_block(
              redeclare package Medium = ThermoCycle.Media.R407c_CP)
            annotation (Placement(transformation(extent={{70,24},{80,34}})));
          Modelica.Blocks.Sources.Ramp ramp1(
            height=0.1,
            duration=0,
            offset=0.45,
            startTime=75)
            annotation (Placement(transformation(extent={{-86,-52},{-76,-42}})));
          Modelica.Blocks.Sources.Constant DELTAT_SP(k=5)
            annotation (Placement(transformation(extent={{74,12},{80,18}})));
          ThermoCycle.Components.Units.ControlSystems.PID PID_valve(
            CSmax=1,
            PVmax=25,
            PVmin=-5,
            CSmin=0,
            CSstart=0.45,
            PVstart=2,
            steadyStateInit=false,
            Kp=-0.9,
            Ti=1)
            annotation (Placement(transformation(extent={{90,26},{108,12}})));
        equation
          connect(sourceMdot1.flangeB, condenser.inlet_fl2)
                                                          annotation (Line(
              points={{-47,56},{-32,56},{-32,35},{-12.8,35}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(condenser.outlet_fl2, sinkP1.flangeB)
                                                      annotation (Line(
              points={{6.8,34.8},{24,34.8},{24,54},{37.6,54}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(condenser.outlet_fl1, tank_pL.InFlow)
                                                      annotation (Line(
              points={{-13,24},{-40,24},{-40,14.4}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(tank_pL.OutFlow, valve.InFlow) annotation (Line(
              points={{-40,-2.8},{-40,-17}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(evaporator.inlet_fl1, valve.OutFlow) annotation (Line(
              points={{-9,-52},{-40,-52},{-40,-35}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sourceMdot2.flangeB, evaporator.inlet_fl2) annotation (Line(
              points={{29,-80},{20,-80},{20,-63},{10.8,-63}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(evaporator.outlet_fl2, sinkP3.flangeB) annotation (Line(
              points={{-8.8,-62.8},{-20,-62.8},{-20,-76},{-31.6,-76}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(electricDrive.shaft, compressor.flange_elc) annotation (Line(
              points={{26.6,-16},{36.4667,-16},{36.4667,-16},{46.3333,-16}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(evaporator.outlet_fl1, dp_ev.InFlow) annotation (Line(
              points={{11,-52},{19,-52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(compressor.OutFlow, dp_cd.InFlow) annotation (Line(
              points={{45.3833,-10},{44,-10},{44,24},{37,24}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(dp_cd.OutFlow, condenser.inlet_fl1) annotation (Line(
              points={{19,24},{7,24}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(ramp.y, electricDrive.f) annotation (Line(
              points={{-1.5,5},{17.6,5},{17.6,-6.6}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dp_ev.OutFlow, sensTp.InFlow) annotation (Line(
              points={{37,-52},{42,-52},{42,-51.84},{48.4,-51.84}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sensTp.OutFlow, compressor.InFlow) annotation (Line(
              points={{59.6,-51.84},{69.7667,-51.84},{69.7667,-27.7}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sensTp.T, sH_block.T_measured) annotation (Line(
              points={{60.4,-43.2},{82,-43.2},{82,4},{62,4},{62,31.5},{69.7,31.5}},
              color={0,0,127},
              smooth=Smooth.None,
              pattern=LinePattern.Dot));
          connect(sensTp.p, sH_block.p_measured) annotation (Line(
              points={{47.6,-43.2},{44,-43.2},{44,-36},{84,-36},{84,6},{64,6},{64,27},{
                  69.9,27}},
              color={0,0,127},
              smooth=Smooth.None,
              pattern=LinePattern.Dot));
          connect(DELTAT_SP.y,PID_valve. SP) annotation (Line(
              points={{80.3,15},{85.15,15},{85.15,16.2},{90,16.2}},
              color={0,0,127},
              smooth=Smooth.None,
              pattern=LinePattern.Dot));
          connect(sH_block.DeltaT, PID_valve.PV) annotation (Line(
              points={{80.55,29.25},{85.275,29.25},{85.275,21.8},{90,21.8}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(PID_valve.CS, valve.cmd) annotation (Line(
              points={{108.54,19},{118,19},{118,82},{-82,82},{-82,-26},{-48,-26}},
              color={0,0,127},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                    -100},{140,100}}),      graphics),
            experiment(StopTime=100),
            __Dymola_experimentSetupOutput,
            Icon(coordinateSystem(extent={{-100,-100},{140,100}})));
        end HeatPump_R407c;
      end Plants;

      package failure_examples "this package proposes practical examples of simulation failures, and their solutions"
      extends Modelica.Icons.Package;

        model Chattering
          "In this model, the simulation fails after a few hundred second. Setting the max derivative to 40 in the evaporator solves the problem"
          Modelica.SIunits.Mass m_wf "Total mass of the working fluid in the cycle";
        ThermoCycle.Components.Units.HeatExchangers.Hx1DConst evaporator(
            N=10,
            redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
            steadystate_T_sf=false,
            steadystate_h_wf=false,
            steadystate_T_wall=false,
            Unom_sf=335,
            max_drhodt_wf=40,
            filter_dMdt_wf=false,
            max_der_wf=false)
            annotation (Placement(transformation(extent={{-62,46},{-34,70}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot(
            cp=1978,
            rho=928.2,
            Mdot_0=3,
            T_0=418.15)
            annotation (Placement(transformation(extent={{-26,72},{-6,92}})));
         ThermoCycle.Components.Units.PdropAndValves.DP dp_hp(
            A=(2*137*77609.9)^(-0.5),
            k=11857.8*137,
            Mdot_nom=0.2588,
            t_init=500,
            redeclare package Medium = ThermoCycle.Media.R245fa_CP,
            constinit=false,
            use_rho_nom=true,
            p_nom=2357000,
            T_nom=413.15,
            DELTAp_lin_nom=3000,
            DELTAp_quad_nom=5150,
            UseHomotopy=false)
            annotation (Placement(transformation(extent={{0,42},{20,62}})));
        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                                expander(
            ExpType=ThermoCycle.Functions.Enumerations.ExpTypes.ORCNext,
            V_s=1,
            constPinit=false,
            constinit=false,
            p_su_start=2357000,
            p_ex_start=177800,
            T_su_start=413.15)
            annotation (Placement(transformation(extent={{24,20},{56,52}})));
          Modelica.Blocks.Sources.Ramp N_rot(
            startTime=50,
            duration=0,
            height=0,
            offset=48.25)  annotation (Placement(transformation(
                extent={{-5,-5},{5,5}},
                rotation=0,
                origin={65,67})));
        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                     generatorNext(Np=1)
            annotation (Placement(transformation(extent={{70,20},{98,48}})));
        ThermoCycle.Components.Units.HeatExchangers.Hx1D    recuperator(
            N=10,
            steadystate_h_cold=true,
            steadystate_h_hot=true,
            steadystate_T_wall=true,
            pstart_hot=177800)
            annotation (Placement(transformation(extent={{-16,15},{16,-15}},
                rotation=90,
                origin={-13,-6})));
        ThermoCycle.Components.Units.PdropAndValves.DP dp_lp(
            k=38.4E3*9.5,
            A=(2*9.5*23282.7)^(-0.5),
            Mdot_nom=0.2588,
            use_rho_nom=true,
            p_nom=190000,
            T_nom=351.15,
            DELTAp_lin_nom=3000,
            DELTAp_quad_nom=5150,
            UseHomotopy=false)
            annotation (Placement(transformation(extent={{32,0},{12,20}})));
        ThermoCycle.Components.Units.HeatExchangers.Hx1DConst condenser(
            Unom_l=500,
            Unom_tp=1500,
            Unom_v=750,
            Mdotnom_sf=4,
            steadystate_T_wall=false,
            N=10,
            max_der_wf=true,
            filter_dMdt_wf=false,
            max_drhodt_wf=50,
            steadystate_T_sf=false,
            steadystate_h_wf=true,
            Unom_sf=335,
            pstart_wf=177800,
            Tstart_inlet_wf=316.92,
            Tstart_outlet_wf=298.15,
            Tstart_inlet_sf=293.15,
            Tstart_outlet_sf=296.36)
            annotation (Placement(transformation(extent={{30,-50},{6,-70}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot heat_sink(
            cp=4187,
            rho=1000,
            Mdot_0=4,
            T_0=293.15)
            annotation (Placement(transformation(extent={{34,-82},{20,-68}})));
        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump
                                                              pump(
            PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.ORCNext,
            PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.freq,
            hstart=2.27e5,
            M_dot_start=0.2588)
            annotation (Placement(transformation(extent={{-74,-54},{-50,-30}})));
            Modelica.Blocks.Sources.Sine sine(
            amplitude=4,
            freqHz=0.05,
            offset=30,
            startTime=50)
            annotation (Placement(transformation(extent={{-94,-14},{-82,-2}})));
         ThermoCycle.Components.Units.Tanks.Tank_pL tank(
            Vtot=0.015,
            L_start=0.5,
            SteadyState_p=false,
            impose_pressure=true,
            pstart=135000)
           annotation (Placement(transformation(extent={{-42,-78},{-24,-60}})));
        equation
        m_wf = 0;
          connect(source_Cdot.flange, evaporator.inletSf)
                                                       annotation (Line(
              points={{-7.8,81.9},{12,81.9},{12,64},{-34,64}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(evaporator.outletWf, dp_hp.InFlow)
                                                 annotation (Line(
              points={{-34,52},{1,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(expander.flange_elc,generatorNext. shaft) annotation (Line(
              points={{50.6667,37.3333},{62.4,37.3333},{62.4,34},{71.96,34}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(N_rot.y,generatorNext. f) annotation (Line(
              points={{70.5,67},{84.56,67},{84.56,47.16}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dp_hp.OutFlow, expander.InFlow) annotation (Line(
              points={{19,52},{30.9333,52},{30.9333,42.1333}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(expander.OutFlow, dp_lp.InFlow) annotation (Line(
              points={{52,28},{52,10},{31,10}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(heat_sink.flange,condenser. inletSf) annotation (Line(
              points={{21.26,-75.07},{-10,-75.07},{-10,-65},{6,-65}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(tank.OutFlow, pump.InFlow) annotation (Line(
              points={{-33,-76.92},{-33,-80},{-78,-80},{-78,-41.4},{-70.64,-41.4}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(tank.InFlow, condenser.outletWf) annotation (Line(
              points={{-33,-61.44},{-33,-55},{6,-55}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.inlet_fl1, pump.OutFlow) annotation (Line(
              points={{-18,-16.6667},{-18,-33.12},{-55.28,-33.12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.outlet_fl1, evaporator.inletWf)
                                                             annotation (Line(
              points={{-18,4.66667},{-18,16},{-76,16},{-76,52},{-62,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.inlet_fl2, dp_lp.OutFlow) annotation (Line(
              points={{-7,4.45333},{-7,10},{13,10}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(recuperator.outlet_fl2, condenser.inletWf) annotation (Line(
              points={{-7.2,-16.4533},{-7.2,-36},{44,-36},{44,-55},{30,-55}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sine.y, pump.flow_in) annotation (Line(
              points={{-81.4,-8},{-65.84,-8},{-65.84,-32.4}},
              color={0,0,127},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                  preserveAspectRatio=false),
                              graphics), Icon(coordinateSystem(extent={{-100,-100},
                    {100,100}})),
            experiment(StopTime=1000),
            __Dymola_experimentSetupOutput);
        end Chattering;
      end failure_examples;
    annotation (Documentation(info="<HTML>
<p><big><dl><dt><b>Main Authors:</b> <br/></dt>
<dd>Sylvain Quoilin; &LT;<a href=\"squoilin@ulg.ac.be\">squoilin@ulg.ac.be</a>&GT;</dd>
<dd>Adriano Desideri &LT;<a href=\"adesideri@ulg.ac.be\">adesideri@ulg.ac.be</a>&GT;<br/></dd>
<dd>University of Liege</dd>
<dd>Laboratory of thermodynamics</dd>
<dd>Campus du Sart-Tilman Bât B49 (P33)</dd>
<dd>B-4000 Liège - BELGIUM -<br/></dd>
<dt><b>Copyright:</b> </dt>
<dd>Copyright &copy; 2013-2014, Sylvain Quoilin and Adriano Desideri.<br/></dd>
<dd><i>The IndustrialControlSystems package is <b>free</b> software; it can be redistributed and/or modified under the terms of the <b>Modelica license</b>.</i><br/></dd>
</dl></html>"));
    end Simulations;

    package TestComponents
      extends Modelica.Icons.Package;

      package Sizing
        extends Modelica.Icons.Package;
        model Sizing_evaporator

          ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot3_2(
            Mdot_0=2,
            T_0=653.15,
            cp=2046,
            rho=1000)
            annotation (Placement(transformation(extent={{58,40},{38,60}})));
          ThermoCycle.Components.Units.HeatExchangers.Hx1DConst eva(
            Mdotconst_wf=false,
            steadystate_h_wf=true,
            steadystate_T_wall=true,
            steadystate_T_sf=false,
            Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
            N=30,
            redeclare package Medium1 = ThermoCycle.Media.Water,
            V_sf=0.0314159,
            V_wf=0.0314159,
            Unom_sf=20000,
            redeclare model Medium1HeatTransferModel =
                ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
            Unom_l=3000,
            Unom_tp=10000,
            Unom_v=3000,
            M_wall=9.35E+01,
            c_wall=385,
            Mdotnom_sf=2,
            Mdotnom_wf=0.05,
            Tstart_inlet_wf=510.97 + 50,
            Tstart_outlet_wf=585.97 + 35,
            Tstart_inlet_sf=360 + 273.15,
            pstart_wf=6000000,
            Tstart_outlet_sf=625.15)
            annotation (Placement(transformation(extent={{-28,0},{8,38}})));

          ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
            Mdot_0=0.05,
            redeclare package Medium = ThermoCycle.Media.Water,
            p=6000000,
            UseT=false,
            T_0=356.26,
            h_0=852450)
            annotation (Placement(transformation(extent={{-90,-20},{-70,0}})));
          ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
              package Medium =
                       ThermoCycle.Media.Water, p0=6000000)
            annotation (Placement(transformation(extent={{76,-6},{96,14}})));
          Modelica.Blocks.Sources.Constant Hin(k=852540)
            annotation (Placement(transformation(extent={{-106,24},{-86,44}},
                  rotation=0)));
        equation
          connect(eva.inletSf, source_Cdot3_2.flange) annotation (Line(
              points={{8,28.5},{16,28.5},{16,30},{24,30},{24,49.9},{39.8,49.9}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(sourceMdot.flangeB, eva.inletWf) annotation (Line(
              points={{-71,-10},{-66,-10},{-66,6},{-54,6},{-54,9.5},{-28,9.5}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Hin.y, sourceMdot.in_h) annotation (Line(
              points={{-85,34},{-74,34},{-74,-4}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(eva.outletWf, sinkP.flangeB) annotation (Line(
              points={{8,9.5},{46,9.5},{46,4},{77.6,4}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(graphics),
            experiment(StopTime=1000),
            __Dymola_experimentSetupOutput);
        end Sizing_evaporator;
      end Sizing;

      model Cell1D

        ThermoCycle.Components.FluidFlow.Pipes.Cell1Dim flow1Dim(
          Ai=0.2,
          Unom_l=400,
          Unom_tp=1000,
          Unom_v=400,
          max_drhodt=50,
          Vi=0.005,
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdotnom=0.3335,
          hstart=84867,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind,

          pstart=866735)
          annotation (Placement(transformation(extent={{-24,8},{-4,28}})));
        ThermoCycle.Components.HeatFlow.Sources.Source_T_cell source_T
          annotation (Placement(transformation(extent={{-24,44},{-4,64}})));
        Modelica.Blocks.Sources.Constant const(k=273.15 + 140)
          annotation (Placement(transformation(extent={{-66,72},{-46,92}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdot_0=0.3335,
          UseT=false,
          h_0=84867,
          p=888343,
          T_0=356.26)
          annotation (Placement(transformation(extent={{-90,12},{-70,32}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          h=254381,
          p0=866735)
          annotation (Placement(transformation(extent={{40,4},{60,24}})));
        Modelica.Blocks.Sources.Sine sine1(
          startTime=10,
          offset=0.3335,
          amplitude=0.5,
          phase=0,
          freqHz=0.1)
          annotation (Placement(transformation(extent={{-108,44},{-94,58}})));
      equation
        connect(const.y, source_T.Temperature) annotation (Line(
            points={{-45,82},{-28,82},{-28,80},{-13,80},{-13,59}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(flow1Dim.Wall_int, source_T.ThermalPortCell) annotation (Line(
            points={{-14,23},{-14,50.9},{-13.1,50.9}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(sourceMdot1.flangeB, flow1Dim.InFlow) annotation (Line(
            points={{-71,22},{-40,22},{-40,18},{-24,18}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, sinkP.flangeB) annotation (Line(
            points={{-4,18.1},{16,18.1},{16,16},{41.6,16},{41.6,14}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sine1.y, sourceMdot1.in_Mdot) annotation (Line(
            points={{-93.3,51},{-86,51},{-86,28}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{
                  100,100}}), graphics),
          experiment(StopTime=50),
          __Dymola_experimentSetupOutput);
      end Cell1D;

      model Flow1Dinc
      parameter Integer N = 10;
      ThermoCycle.Components.FluidFlow.Pipes.Flow1DimInc       HotFluid(
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
          A=16.18,
          V=0.03781,
          Unom=30.21797814,
          Mdotnom=0.25877,
          N=N,
          steadystate=false,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.centr_diff,
          pstart=100000,
          Tstart_inlet=418.15,
          Tstart_outlet=408.15)
          annotation (Placement(transformation(extent={{-20,94},{28,42}})));
          //Tstart_inlet=402.157968,
          //Tstart_outlet=357.76567852)
          //Tstart_inlet=353.82,
          //Tstart_outlet=316.91)
       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot
                 sourceWF1(
          h_0=470523,
          Mdot_0=3,
          UseT=true,
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
          p=100000,
          T_0=418.15)
          annotation (Placement(transformation(extent={{-90,62},{-70,82}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP      sinkPFluid1(redeclare
            package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66, p0=
              100000)
          annotation (Placement(transformation(extent={{74,50},{94,70}})));
        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-54,10},{-34,30}})));
        Modelica.Blocks.Sources.Constant const(k=25 + 273.15)
          annotation (Placement(transformation(extent={{-92,18},{-72,38}})));
      equation
        connect(source_T.thermalPort, HotFluid.Wall_int) annotation (Line(
            points={{-44.1,15.9},{-44.1,0},{4,0},{4,57.1667}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(const.y, source_T.Temperature) annotation (Line(
            points={{-71,28},{-44,28},{-44,24}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sourceWF1.flangeB, HotFluid.InFlow) annotation (Line(
            points={{-71,72},{-60,72},{-60,76},{-44,76},{-44,68},{-16,68}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(HotFluid.OutFlow, sinkPFluid1.flangeB) annotation (Line(
            points={{24,67.7833},{40,67.7833},{40,60},{75.6,60}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(graphics),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Flow1Dinc;

      model Flow1DConst
      parameter Integer N = 5;
      ThermoCycle.Components.FluidFlow.Pipes.Flow1DConst flowConst(N=N)
          annotation (Placement(transformation(extent={{20,94},{-42,44}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot
                                                     source_Cdot(
          cp=1978,
          rho=928.2,
          Mdot_0=3,
          T_0=418.15)
          annotation (Placement(transformation(extent={{50,68},{70,88}})));
        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-52,-4},{-32,16}})));
        Modelica.Blocks.Sources.Constant const(k=25 + 273.15)
          annotation (Placement(transformation(extent={{-90,4},{-70,24}})));
      equation
        connect(source_Cdot.flange, flowConst.flange_Cdot) annotation (Line(
            points={{68.2,77.9},{74,77.9},{74,69},{20,69}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(const.y,source_T. Temperature) annotation (Line(
            points={{-69,14},{-42,14},{-42,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(source_T.thermalPort, flowConst.Wall_int) annotation (Line(
            points={{-42.1,1.9},{-42.1,-10},{-11,-10},{-11,56.5}},
            color={255,0,0},
            smooth=Smooth.None));
        annotation (Diagram(graphics),
          experiment(StopTime=100),
          __Dymola_experimentSetupOutput);
      end Flow1DConst;

      model flow1D
        parameter Integer N = 10;
        ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim flow1Dim(
          A=2,
          Unom_l=400,
          Unom_v=400,
          N=N,
          V=0.003,
          Mdotnom=0.3,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          redeclare model Flow1DimHeatTransferModel =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
          Unom_tp=400,
          pstart=1000000,
          Tstart_inlet=323.15,
          Tstart_outlet=373.15)
          annotation (Placement(transformation(extent={{-40,-4},{-2,34}})));

        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-30,42},{4,64}})));
        Modelica.Blocks.Sources.Constant const(k=273.15 + 140)
          annotation (Placement(transformation(extent={{-62,64},{-54,72}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdot_0=0.3335,
          UseT=false,
          h_0=84867,
          p=888343,
          T_0=356.26)
          annotation (Placement(transformation(extent={{-102,6},{-76,32}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          h=254381,
          p0=866735)
          annotation (Placement(transformation(extent={{50,14},{70,34}})));
        Modelica.Blocks.Sources.Sine sine(
          startTime=10,
          offset=0.3335,
          amplitude=0.5,
          phase=0,
          freqHz=0.1)
          annotation (Placement(transformation(extent={{-116,50},{-102,64}})));
        ThermoCycle.Components.FluidFlow.Sensors.SensTp T_su_Sensor(redeclare
            package Medium = CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{-66,10},{-46,30}})));
        ThermoCycle.Components.FluidFlow.Sensors.SensTp T_ex_Sensor(redeclare
            package Medium = CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{14,14},{34,34}})));
      equation
        connect(source_T.thermalPort, flow1Dim.Wall_int) annotation (Line(
            points={{-13.17,48.49},{-13.17,43.95},{-21,43.95},{-21,22.9167}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(const.y, source_T.Temperature) annotation (Line(
            points={{-53.6,68},{-13,68},{-13,57.4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sine.y, sourceMdot1.in_Mdot) annotation (Line(
            points={{-101.3,57},{-98.5,57},{-98.5,26.8},{-96.8,26.8}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, T_ex_Sensor.InFlow) annotation (Line(
            points={{-5.16667,15.1583},{4,15.1583},{4,19.2},{17,19.2}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(T_ex_Sensor.OutFlow, sinkP.flangeB) annotation (Line(
            points={{31,19.2},{38,19.2},{38,24},{51.6,24}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-120,-100},{80,
                  100}}),     graphics={Text(
                extent={{-62,56},{-26,50}},
                lineColor={0,0,0},
                textString="Thermal port")}),
          experiment(StopTime=50),
          __Dymola_experimentSetupOutput,
          Icon(coordinateSystem(extent={{-120,-100},{80,100}})));
      end flow1D;

      model flow1D_reversal
        parameter Integer N = 10;
        ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim flow1Dim(
          A=2,
          Unom_l=400,
          Unom_tp=1000,
          Unom_v=400,
          N=N,
          V=0.003,
          Mdotnom=0.3,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_smooth,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          pstart=500000,
          Tstart_inlet=323.15,
          Tstart_outlet=373.15)
          annotation (Placement(transformation(extent={{-22,16},{-2,36}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
          Mdot_0=0.3,
          UseT=false,
          h_0=2E5,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          p=500000,
          T_0=293.15)
          annotation (Placement(transformation(extent={{-80,16},{-60,36}})));
        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-24,48},{-4,68}})));
        Modelica.Blocks.Sources.Constant const(k=273.15 + 140)
          annotation (Placement(transformation(extent={{-66,72},{-46,92}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(h=4e5, redeclare
            package Medium = ThermoCycle.Media.R245fa_CP)
          annotation (Placement(transformation(extent={{34,16},{54,36}})));
        Modelica.Blocks.Sources.Ramp ramp(
          offset=5E5,
          duration=5,
          startTime=5,
          height=8E5) annotation (Placement(transformation(extent={{14,56},{34,76}})));
        Modelica.Blocks.Sources.Ramp ramp1(
          duration=5,
          startTime=5,
          offset=0.3,
          height=-0.6)
                      annotation (Placement(transformation(extent={{-112,44},{-92,64}})));
      equation
        connect(sourceMdot.flangeB, flow1Dim.InFlow) annotation (Line(
            points={{-61,26},{-20.3333,26}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(source_T.thermalPort, flow1Dim.Wall_int) annotation (Line(
            points={{-14.1,53.9},{-14.1,43.95},{-12,43.95},{-12,30.1667}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(const.y, source_T.Temperature) annotation (Line(
            points={{-45,82},{-28,82},{-28,80},{-14,80},{-14,62}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, sinkP.flangeB) annotation (Line(
            points={{-3.66667,26.0833},{12,26.0833},{12,26},{35.6,26}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, sinkP.in_p0) annotation (Line(
            points={{35,66},{40,66},{40,34.8}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(ramp1.y, sourceMdot.in_Mdot) annotation (Line(
            points={{-91,54},{-76,54},{-76,32}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}),     graphics),
          experiment(StopTime=50, __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput);
      end flow1D_reversal;

      model flow1D_reverse
        parameter Integer N = 10;
        ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim flow1Dim(
          A=2,
          Unom_l=400,
          Unom_tp=1000,
          Unom_v=400,
          N=N,
          V=0.003,
          Mdotnom=0.3,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.centr_diff_AllowFlowReversal,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          pstart=500000,
          Tstart_inlet=323.15,
          Tstart_outlet=373.15)
          annotation (Placement(transformation(extent={{-22,16},{-2,36}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
          UseT=false,
          h_0=2E5,
          Mdot_0=-0.3,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          p=500000,
          T_0=293.15)
          annotation (Placement(transformation(extent={{-80,16},{-60,36}})));
        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-24,48},{-4,68}})));
        Modelica.Blocks.Sources.Constant const(k=273.15 + 40)
          annotation (Placement(transformation(extent={{-66,72},{-46,92}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(h=4e5, redeclare
            package Medium = ThermoCycle.Media.R245fa_CP)
          annotation (Placement(transformation(extent={{34,16},{54,36}})));
        Modelica.Blocks.Sources.Ramp ramp(
          offset=5E5,
          duration=5,
          startTime=5,
          height=-1E5)
                      annotation (Placement(transformation(extent={{14,56},{34,76}})));
      equation
        connect(sourceMdot.flangeB, flow1Dim.InFlow) annotation (Line(
            points={{-61,26},{-20.3333,26}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(source_T.thermalPort, flow1Dim.Wall_int) annotation (Line(
            points={{-14.1,53.9},{-14.1,43.95},{-12,43.95},{-12,30.1667}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(const.y, source_T.Temperature) annotation (Line(
            points={{-45,82},{-28,82},{-28,80},{-14,80},{-14,62}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, sinkP.flangeB) annotation (Line(
            points={{-3.66667,26.0833},{12,26.0833},{12,26},{35.6,26}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, sinkP.in_p0) annotation (Line(
            points={{35,66},{40,66},{40,34.8}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{
                  100,100}}), graphics),
          experiment(StopTime=50, __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput);
      end flow1D_reverse;

      model flow1D_smoothed
        parameter Integer N = 11;
        replaceable package Medium = CoolProp2Modelica.Media.R134a_CP(substanceNames={"R134a|calc_transport=1|enable_TTSE=0"})
        constrainedby Modelica.Media.Interfaces.PartialMedium;
        ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim flow1Dim(
          N=N,
          redeclare package Medium = Medium,
          Mdotnom=0.2,
          A=0.25,
          V=0.002,
          Unom_tp=3000,
          filter_dMdt=true,
          max_der=true,
          Unom_l=1500,
          Unom_v=1500,
          pstart=500000,
          Tstart_inlet=218.15,
          Tstart_outlet=328.15,
          redeclare model Flow1DimHeatTransferModel =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.SmoothedInit
              (
              max_dUdt=0,
              redeclare model TwoPhaseCorrelation =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.TwoPhaseCorrelations.Constant,
              redeclare model LiquidCorrelation =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.SinglePhaseCorrelations.DittusBoelter
                  (d_hyd=2*flow1Dim.V/flow1Dim.A),
              redeclare model VapourCorrelation =
                  ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.SinglePhaseCorrelations.DittusBoelter
                  (d_hyd=2*flow1Dim.V/flow1Dim.A)))
          annotation (Placement(transformation(extent={{-20,-20},{20,20}})));

        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          redeclare package Medium = Medium,
          UseT=false,
          h_0=1.3e5,
          Mdot_0=0.15,
          p=500000,
          T_0=356.26)
          annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(
          redeclare package Medium = Medium,
          h=254381,
          p0=500000)
          annotation (Placement(transformation(extent={{60,-10},{80,10}})));
        ThermoCycle.Components.FluidFlow.Sources.SourceMdot2 sourceMdot2_1
          annotation (Placement(transformation(extent={{-98,12},{-78,32}})));
        ThermoCycle.Interfaces.HeatTransfer.HeatPortConverter heatPortConverter
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={0,76})));
        Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow fixedHeatFlow(
          alpha=50,
          Q_flow=50000,
          T_ref=403.15)
          annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
        ThermoCycle.Components.HeatFlow.Walls.MetalWall metalWall(
          N=N,
          Aext=1.5*flow1Dim.A,
          Aint=flow1Dim.A,
          M_wall=1,
          c_wall=500,
          Tstart_wall_1=fixedHeatFlow.T_ref,
          Tstart_wall_end=fixedHeatFlow.T_ref,
          steadystate_T_wall=false)
          annotation (Placement(transformation(extent={{-20,48},{20,8}})));
        ThermoCycle.Interfaces.HeatTransfer.ThermalPortMultiplier thermalPortMultiplier(N=N)
          annotation (Placement(transformation(extent={{-10,60},{10,40}})));
      equation
        connect(sourceMdot1.flangeB, flow1Dim.InFlow) annotation (Line(
            points={{-61,0},{-16.6667,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, sinkP.flangeB) annotation (Line(
            points={{16.6667,0.166667},{18,0.166667},{18,0},{61.6,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sourceMdot2_1.y, sourceMdot1.in_Mdot) annotation (Line(
            points={{-79,23},{-76,23},{-76,6}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(fixedHeatFlow.port, heatPortConverter.heatPort) annotation (Line(
            points={{-40,80},{-20,80},{-20,86},{1.83697e-015,86}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(metalWall.Wall_int, flow1Dim.Wall_int) annotation (Line(
            points={{0,22},{0,8.33333}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(heatPortConverter.thermalPortL, thermalPortMultiplier.single)
          annotation (Line(
            points={{-1.83697e-015,66},{-1.83697e-015,60.05},{0,60.05},{0,54.1}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(thermalPortMultiplier.multi, metalWall.Wall_ext) annotation (Line(
            points={{0,46.5},{0,34},{-0.4,34}},
            color={255,0,0},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                  100,100}}), graphics),
          experiment(
            StopTime=500,
            __Dymola_NumberOfIntervals=1000,
            __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput);
      end flow1D_smoothed;

      model flow1D_MD
        parameter Integer N = 10;
      ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim_MD  flow1Dim(
          A=2,
          U_nom = 400,
          N=N,
          V=0.003,
          Mdotnom=0.3,
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          pstart=1000000,
          Tstart_inlet=323.15,
          Tstart_outlet=373.15)
          annotation (Placement(transformation(extent={{-32,2},{6,40}})));

        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdot_0=0.3335,
          UseT=false,
          h_0=84867,
          p=888343,
          T_0=356.26)
          annotation (Placement(transformation(extent={{-92,6},{-66,32}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          h=254381,
          p0=866735)
          annotation (Placement(transformation(extent={{50,14},{70,34}})));
        Modelica.Blocks.Sources.Sine sine(
          startTime=10,
          offset=0.3335,
          amplitude=0.5,
          phase=0,
          freqHz=0.1)
          annotation (Placement(transformation(extent={{-116,50},{-102,64}})));
        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-20,52},{14,74}})));
        Modelica.Blocks.Sources.Constant const(k=273.15 + 140)
          annotation (Placement(transformation(extent={{-52,74},{-44,82}})));
        ThermoCycle.Components.FluidFlow.Sensors.SensTp T_su_Sensor(redeclare
            package Medium = CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{-60,12},{-40,32}})));
        ThermoCycle.Components.FluidFlow.Sensors.SensTp T_ex_Sensor(redeclare
            package Medium = CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{18,16},{38,36}})));
      equation
        connect(sine.y, sourceMdot1.in_Mdot) annotation (Line(
            points={{-101.3,57},{-86.8,57},{-86.8,26.8}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(const.y,source_T. Temperature) annotation (Line(
            points={{-43.6,78},{-3,78},{-3,67.4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(source_T.thermalPort, flow1Dim.Wall_int) annotation (Line(
            points={{-3.17,58.49},{-3.17,48},{-16,48},{-16,28.9167},{-13,
                28.9167}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(sourceMdot1.flangeB, T_su_Sensor.InFlow) annotation (Line(
            points={{-67.3,19},{-59.65,19},{-59.65,17.2},{-57,17.2}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(T_su_Sensor.OutFlow, flow1Dim.InFlow) annotation (Line(
            points={{-43,17.2},{-37.5,17.2},{-37.5,21},{-28.8333,21}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, T_ex_Sensor.InFlow) annotation (Line(
            points={{2.83333,21.1583},{12.4167,21.1583},{12.4167,21.2},{21,21.2}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(T_ex_Sensor.OutFlow, sinkP.flangeB) annotation (Line(
            points={{35,21.2},{44.5,21.2},{44.5,24},{51.6,24}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{
                  100,100}}), graphics={Text(
                extent={{-52,66},{-16,60}},
                lineColor={0,0,0},
                textString="Thermal port")}),
          experiment(StopTime=125, __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput);
      end flow1D_MD;

      model DP

      ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          Mdot_0=-0.6,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          p=400000,
          T_0=298.15)
          annotation (Placement(transformation(extent={{-84,30},{-64,50}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
            package                                                                    Medium =
                             ThermoCycle.Media.R245fa_CP,  p0=400000)
          annotation (Placement(transformation(extent={{24,32},{44,52}})));
        ThermoCycle.Components.Units.PdropAndValves.DP dP(
          h=3,
          UseHomotopy=true,
          A=4e-5,
          constinit=true,
          Mdot_nom=0.5,
          UseNom=true,
          t_init=0.5,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          p_nom=400000,
          T_nom=298.15,
          DELTAp_stat_nom=40000,
          DELTAp_lin_nom=5000,
          DELTAp_quad_nom=60000)
          annotation (Placement(transformation(extent={{-30,32},{-10,52}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=1,
          duration=2,
          startTime=1,
          offset=0.5)
          annotation (Placement(transformation(extent={{-104,78},{-84,98}})));
        Real y;
      equation
        connect(ramp.y, sourceWF.in_Mdot) annotation (Line(
            points={{-83,88},{-60,88},{-60,58},{-80,58},{-80,46}},
            color={0,0,127},
            smooth=Smooth.None));
        y = ThermoCycle.Functions.weightingfactor(
              t_init=2,
              length=5,
              t=time)
        annotation (Diagram(graphics), uses(Modelica(version="3.2")));
        connect(sourceWF.flangeB, dP.InFlow) annotation (Line(
            points={{-65,40},{-58,40},{-58,38},{-50,38},{-50,42},{-29,42}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(dP.OutFlow, sinkPFluid.flangeB) annotation (Line(
            points={{-11,42},{0,42},{0,36},{14,36},{14,42},{25.6,42}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(graphics),
          experiment(StopTime=50),
          __Dymola_experimentSetupOutput);
      end DP;

      model Expander

        Modelica.Blocks.Sources.Ramp N_rot(
          duration=100,
          startTime=400,
          offset=48.25,
          height=0)      annotation (Placement(transformation(
              extent={{-5,-5},{5,5}},
              rotation=0,
              origin={-13,77})));
       ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                               expander(redeclare
            package Medium = ThermoCycle.Media.R245fa_CP,
                                               V_s=1)
          annotation (Placement(transformation(extent={{-32,16},{-12,36}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
            package Medium = ThermoCycle.Media.R245fa_CP,  p0=153400)
          annotation (Placement(transformation(extent={{8,-10},{28,10}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          Mdot_0=0.2588,
          UseT=false,
          h_0=503925,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP)
          annotation (Placement(transformation(extent={{-62,40},{-42,60}})));
       ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                generator
          annotation (Placement(transformation(extent={{16,28},{36,48}})));
      equation
        connect(sourceWF.flangeB, expander.InFlow) annotation (Line(
            points={{-43,50},{-32,50},{-32,29.8333},{-27.6667,29.8333}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(expander.OutFlow, sinkPFluid.flangeB) annotation (Line(
            points={{-14.5,21},{-6,21},{-6,0},{9.6,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(expander.flange_elc, generator.shaft) annotation (Line(
            points={{-15.3333,26.8333},{8,26.8333},{8,38},{17.4,38}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(N_rot.y, generator.f) annotation (Line(
            points={{-7.5,77},{26.4,77},{26.4,47.4}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-120,-100},{80,100}},
                preserveAspectRatio=true),
                            graphics), Icon(coordinateSystem(extent={{-120,-100},
                  {80,100}})),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Expander;

      model Hx1DConst

      ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
            package Medium =
                     ThermoCycle.Media.R245fa_CP,  p0=2357000)
          annotation (Placement(transformation(extent={{82,-10},{102,10}})));
      ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          Mdot_0=0.2588,
          h_0=281455,
          UseT=true,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          p=2357000,
          T_0=353.15)
          annotation (Placement(transformation(extent={{-92,-10},{-72,10}})));
       ThermoCycle.Components.Units.HeatExchangers.Hx1DConst hx1DConst(
          redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
          steadystate_T_sf=true,
          steadystate_h_wf=true,
          steadystate_T_wall=true,
          N=10,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind,
          SecondaryFluid(Discretization=ThermoCycle.Functions.Enumerations.Discretizations.centr_diff),
          counterCurrent=false)
          annotation (Placement(transformation(extent={{-30,-2},{2,36}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot   source_Cdot(
          cp=1978,
          rho=928.2,
          Mdot_0=3,
          T_0=418.15)
          annotation (Placement(transformation(extent={{-20,46},{0,66}})));
      equation
        connect(sourceWF.flangeB, hx1DConst.inletWf)
                                                   annotation (Line(
            points={{-73,0},{-66,0},{-66,-2},{-56,-2},{-56,7.5},{-30,7.5}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(hx1DConst.outletWf, sinkPFluid.flangeB)
                                                      annotation (Line(
            points={{2,7.5},{22,7.5},{22,6},{36,6},{36,-2},{83.6,-2},{83.6,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(source_Cdot.flange, hx1DConst.inletSf)
                                                     annotation (Line(
            points={{-1.8,55.9},{36,55.9},{36,26.5},{2,26.5}},
            color={255,0,0},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                preserveAspectRatio=true),
                            graphics), Icon(coordinateSystem(extent={{-100,-100},
                  {100,100}})),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Hx1DConst;

      model Hx1D
      // parameter Real k( start = 1, fixed = false)= 1;
      //
      // Modelica.SIunits.Power PowerRec = Recuperator.Q_hot_;

        ThermoCycle.Components.Units.HeatExchangers.Hx1D    Recuperator(
          redeclare package Medium1 = CoolProp2Modelica.Media.SES36_CP,
          redeclare package Medium2 = CoolProp2Modelica.Media.SES36_CP,
          MdotNom_Hot=0.3335,
          MdotNom_Cold=0.3335,
          steadystate_h_cold=true,
          N=10,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.centr_diff,
          Unom_l_cold=1500,
          Unom_tp_cold=1500,
          Unom_v_cold=1500,
          Unom_l_hot=1000,
          Unom_tp_hot=1000,
          Unom_v_hot=1000,
          pstart_cold=863885,
          pstart_hot=127890,
          Tstart_inlet_cold=303.59,
          Tstart_outlet_cold=356.26,
          Tstart_inlet_hot=368.05,
          Tstart_outlet_hot=315.81)
          annotation (Placement(transformation(extent={{-34,-22},{28,40}})));

        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdot_0=0.3335,
          p=888343,
          T_0=303.59)
          annotation (Placement(transformation(extent={{-94,-28},{-74,-8}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium =
                     CoolProp2Modelica.Media.SES36_CP,
          h=290659,
          p0=863885)
          annotation (Placement(transformation(extent={{68,-48},{88,-28}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdot_0=0.3335,
          UseT=false,
          h_0=236271,
          p=127890,
          T_0=368.05)
          annotation (Placement(transformation(extent={{26,70},{46,90}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          h=363652,
          p0=127890)
          annotation (Placement(transformation(extent={{-54,72},{-36,90}})));
      // initial equation
      // PowerRec = 20.330e3;

        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp(redeclare package
            Medium = CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{-32,18},{-52,38}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp1(redeclare
            package Medium =
                     CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{30,-12},{50,8}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp2(redeclare
            package Medium =
                     CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{-62,-22},{-42,-2}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp3(redeclare
            package Medium =
                     CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{40,34},{60,54}})));
      equation

        connect(Recuperator.outlet_fl1, sensTp1.InFlow) annotation (Line(
            points={{17.6667,-1.33333},{26,-1.33333},{26,-6.8},{33,-6.8}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp1.OutFlow, sinkP.flangeB) annotation (Line(
            points={{47,-6.8},{60,-6.8},{60,-38},{69.6,-38}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp.InFlow, Recuperator.outlet_fl2) annotation (Line(
            points={{-35,23.2},{-30,20.9867},{-23.2533,20.9867}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp.OutFlow, sinkP1.flangeB) annotation (Line(
            points={{-49,23.2},{-66,23.2},{-66,81},{-52.56,81}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sourceMdot.flangeB, sensTp2.InFlow) annotation (Line(
            points={{-75,-18},{-66,-18},{-66,-16.8},{-59,-16.8}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp2.OutFlow, Recuperator.inlet_fl1) annotation (Line(
            points={{-45,-16.8},{-32,-16.8},{-32,-1.33333},{-23.6667,-1.33333}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sourceMdot1.flangeB, sensTp3.InFlow) annotation (Line(
            points={{45,80},{54,80},{54,58},{32,58},{32,39.2},{43,39.2}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp3.OutFlow, Recuperator.inlet_fl2) annotation (Line(
            points={{57,39.2},{68,39.2},{68,21.4},{17.2533,21.4}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}}),
                            graphics), Icon(coordinateSystem(extent={{-100,-100},
                  {100,100}})),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Hx1D;

      model Hx1DInc

        ThermoCycle.Components.Units.HeatExchangers.Hx1DInc hx1DInc(
          Mdotnom_sf=3.148,
          redeclare package Medium1 = ThermoCycle.Media.SES36_CP,
          Mdotnom_wf=0.3335,
          steadystate_h_wf=true,
          N=10,
          Unom_sf=900,
          Mdotconst_wf=false,
          max_der_wf=false,
          Unom_l=3000,
          Unom_tp=3600,
          Unom_v=3000,
          redeclare package Medium2 =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
          pstart_sf=100000,
          pstart_wf=888343,
          Tstart_inlet_wf=356.26,
          Tstart_outlet_wf=397.75,
          Tstart_inlet_sf=398.15,
          Tstart_outlet_sf=389.45)
          annotation (Placement(transformation(extent={{-38,-24},{24,38}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
          redeclare package Medium = ThermoCycle.Media.SES36_CP,
          Mdot_0=0.3335,
          UseT=false,
          h_0=84867,
          p=888343,
          T_0=356.26)
          annotation (Placement(transformation(extent={{-90,-28},{-70,-8}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package                                                               Medium =
                     ThermoCycle.Media.SES36_CP,
          h=254381,
          p0=888343)
          annotation (Placement(transformation(extent={{68,-48},{88,-28}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          Mdot_0=3.148,
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
          p=100000,
          T_0=398.15)
          annotation (Placement(transformation(extent={{26,70},{46,90}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
            package                                                                Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66, p0=
              100000)
          annotation (Placement(transformation(extent={{-54,72},{-36,90}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp(redeclare package
                                                                                Medium =

              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66)
          annotation (Placement(transformation(extent={{-66,22},{-46,42}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp1(redeclare
            package                                                              Medium =

              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66)
          annotation (Placement(transformation(extent={{28,24},{48,44}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp2(redeclare
            package                                                              Medium =
                     ThermoCycle.Media.SES36_CP)
          annotation (Placement(transformation(extent={{-66,-10},{-46,10}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp3(redeclare
            package                                                              Medium =
                     ThermoCycle.Media.SES36_CP)
          annotation (Placement(transformation(extent={{32,0},{52,20}})));
      equation

        connect(hx1DInc.outlet_fl2, sensTp.OutFlow) annotation (Line(points={{
                -30.3692,20.8308},{-40.1846,20.8308},{-40.1846,27.2},{-49,27.2}},
                                             color={0,0,255}));
        connect(sensTp.InFlow, sinkP1.flangeB)
          annotation (Line(points={{-63,27.2},{-63,53.6},{-52.56,53.6},{-52.56,81}}, color={0,0,255}));
        connect(hx1DInc.inlet_fl2, sensTp1.InFlow) annotation (Line(points={{16.3692,
                21.3077},{24.1846,21.3077},{24.1846,29.2},{31,29.2}},
                                           color={0,0,255}));
        connect(sensTp1.OutFlow, sourceMdot1.flangeB)
          annotation (Line(points={{45,29.2},{45,54.6},{45,54.6},{45,80}}, color={0,0,255}));
        connect(hx1DInc.inlet_fl1, sensTp2.OutFlow) annotation (Line(points={{
                -30.8462,-4.92308},{-39.4231,-4.92308},{-39.4231,-4.8},{-49,
                -4.8}},                      color={0,0,255}));
        connect(sensTp2.InFlow, sourceMdot.flangeB)
          annotation (Line(points={{-63,-4.8},{-63,-11.4},{-71,-11.4},{-71,-18}}, color={0,0,255}));
        connect(hx1DInc.outlet_fl1, sensTp3.InFlow) annotation (Line(points={{16.8462,
                -4.92308},{25.4231,-4.92308},{25.4231,5.2},{35,5.2}},
                                         color={0,0,255}));
        connect(sensTp3.OutFlow, sinkP.flangeB)
          annotation (Line(points={{49,5.2},{49,-16.4},{69.6,-16.4},{69.6,-38}}, color={0,0,255}));
        annotation (
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Hx1DInc;

      model MBeva
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
          UseT=false,
          h_0=852450,
          Mdot_0=0.05,
          redeclare package Medium = ThermoCycle.Media.Water,
          p=6000000)
          annotation (Placement(transformation(extent={{-84,-54},{-64,-34}})));
        Modelica.Blocks.Sources.Constant Hin(k=852540)
          annotation (Placement(transformation(extent={{-100,-16},{-80,4}},
                rotation=0)));
        Modelica.Blocks.Sources.Constant Pout(k=1e5)
          annotation (Placement(transformation(extent={{130,10},{110,30}},
                                                                         rotation=
                 0)));
       ThermoCycle.Components.Units.HeatExchangers.MBeva MB_eva(
          A=0.0314159,
          L=1,
          M_tot=9.35E+01,
          c_wall=385,
          rho_wall=8.93e3,
          TwSB(start=510.97 + 50, displayUnit="K"),
          TwTP(start=548.79 + 21, displayUnit="K"),
          TwSH(start=585.97 + 35, displayUnit="K"),
          Tsf_SU_start=360 + 273.15,
          U_SB=3000,
          U_TP=10000,
          U_SH=3000,
          L_SB(start=0.2),
          L_TP(start=0.4),
          h_EX(start=3043000),
          Void=0.665,
          dVoid_dp=0,
          dVoid_dh=0,
          Usf=20000,
          ETA=1,
          redeclare package Medium = ThermoCycle.Media.Water,
          p(start=6000000),
          dTsf_start=343.15)
          annotation (Placement(transformation(extent={{-26,-38},{22,10}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot sourceCdot(
          rho=1000,
          Mdot_0=2,
          cp=2046,
          T_0=653.15) annotation (Placement(transformation(extent={{2,30},{22,50}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium =
              ThermoCycle.Media.Water)
          annotation (Placement(transformation(extent={{88,-46},{108,-26}})));
        ThermoCycle.Components.Units.PdropAndValves.Valve valve(
          UseNom=true,
          redeclare package Medium = ThermoCycle.Media.Water,
          Mdot_nom=0.05,
          p_nom=6000000,
          T_nom=643.15,
          DELTAp_nom=5000000)
          annotation (Placement(transformation(extent={{54,-64},{74,-44}})));
      equation
        connect(Hin.y, sourceMdot.in_h) annotation (Line(
            points={{-79,-6},{-68,-6},{-68,-38}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(Pout.y, sinkP.in_p0) annotation (Line(
            points={{109,20},{94,20},{94,-27.2}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sourceCdot.flange, MB_eva.InFlow_sf) annotation (Line(
            points={{20.2,39.9},{34,39.9},{34,0.4},{22,0.4}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(sourceMdot.flangeB, MB_eva.InFlow) annotation (Line(
            points={{-65,-44},{-58,-44},{-58,-40},{-50,-40},{-50,-29.36},{-25.52,
                -29.36}},
            color={0,0,255},
            smooth=Smooth.None));

        connect(MB_eva.OutFlow, valve.InFlow) annotation (Line(
            points={{22,-29.36},{40,-29.36},{40,-54},{55,-54}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(valve.OutFlow, sinkP.flangeB) annotation (Line(
            points={{73,-54},{80,-54},{80,-40},{82,-40},{82,-36},{89.6,-36}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                preserveAspectRatio=true),
                            graphics), Icon(coordinateSystem(extent={{-100,-100},{100,
                  100}})),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end MBeva;

      model Test_Evaporator_MBComp
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot source_Cdot3_2(
          Mdot_0=2,
          T_0=653.15,
          cp=2046,
          rho=1000)
          annotation (Placement(transformation(extent={{58,40},{38,60}})));
        ThermoCycle.Components.Units.HeatExchangers.Hx1DConst eva(
          Mdotconst_wf=false,
          steadystate_h_wf=true,
          steadystate_T_wall=true,
          steadystate_T_sf=false,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
          N=30,
          redeclare package Medium1 = ThermoCycle.Media.Water,
          V_sf=0.0314159,
          V_wf=0.0314159,
          Unom_sf=20000,
          redeclare model Medium1HeatTransferModel =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
          Unom_l=3000,
          Unom_tp=10000,
          Unom_v=3000,
          M_wall=9.35E+01,
          c_wall=385,
          Mdotnom_sf=2,
          Mdotnom_wf=0.05,
          Tstart_inlet_wf=510.97 + 50,
          Tstart_outlet_wf=585.97 + 35,
          Tstart_inlet_sf=360 + 273.15,
          pstart_wf=6000000,
          Tstart_outlet_sf=625.15)
          annotation (Placement(transformation(extent={{-28,0},{8,38}})));

        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
          Mdot_0=0.05,
          redeclare package Medium = ThermoCycle.Media.Water,
          p=6000000,
          UseT=false,
          T_0=356.26,
          h_0=852450)
          annotation (Placement(transformation(extent={{-90,-20},{-70,0}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium =
                     ThermoCycle.Media.Water, p0=6000000)
          annotation (Placement(transformation(extent={{76,-6},{96,14}})));
        Modelica.Blocks.Sources.Constant Hin(k=852540)
          annotation (Placement(transformation(extent={{-106,24},{-86,44}},
                rotation=0)));
      equation
        connect(eva.inletSf, source_Cdot3_2.flange) annotation (Line(
            points={{8,28.5},{16,28.5},{16,30},{24,30},{24,49.9},{39.8,49.9}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(sourceMdot.flangeB, eva.inletWf) annotation (Line(
            points={{-71,-10},{-66,-10},{-66,6},{-54,6},{-54,9.5},{-28,9.5}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Hin.y, sourceMdot.in_h) annotation (Line(
            points={{-85,34},{-74,34},{-74,-4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(eva.outletWf, sinkP.flangeB) annotation (Line(
            points={{8,9.5},{46,9.5},{46,4},{77.6,4}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-120,-40},{100,60}}),
                            graphics),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput,
          Icon(coordinateSystem(extent={{-120,-40},{100,60}})));
      end Test_Evaporator_MBComp;

      model CrossHx

        ThermoCycle.Components.Units.HeatExchangers.CrossHX crossHX(
          V_wf=1.36e-4,
          A_wf=0.06248,
          V_sf=0,
          A_sf=1.091,
          Mdotnom_wf=0.19/12,
          Mdotnom_sf=6/12/2,
          M_wall_tot=2.85,
          c_wall=660,
          Unom_l=430,
          Unom_tp=4400,
          Unom_v=660,
          Unom_sf=50,
          UseNom_sf=true,
          T_nom_sf=298.15,
          DELTAp_quad_nom_sf=50000,
          pstart_wf=211000,
          Tstart_wf_in=323.15)
          annotation (Placement(transformation(extent={{-44,-26},{34,54}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceR245faCool(
          Mdot_0=0.19/12,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          T_0=323.15)
          annotation (Placement(transformation(extent={{-90,4},{-70,24}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceAir(
          redeclare package Medium = Modelica.Media.Air.SimpleAir,
          Mdot_0=6/12/2,
          T_0=303.81)
          annotation (Placement(transformation(extent={{-10,76},{10,96}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium = ThermoCycle.Media.R245fa_CP, p0=211000)
          annotation (Placement(transformation(extent={{82,12},{102,32}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
            package Medium = Modelica.Media.Air.SimpleAir)
          annotation (Placement(transformation(extent={{8,-56},{28,-36}})));
      equation
        connect(sourceR245faCool.flangeB, crossHX.Inlet_fl1) annotation (Line(
            points={{-71,14},{-44.78,14}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(crossHX.Outlet_fl1, sinkP.flangeB) annotation (Line(
            points={{33.22,14},{58.61,14},{58.61,22},{83.6,22}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sourceAir.flangeB, crossHX.Inlet_fl2) annotation (Line(
            points={{9,86},{14,86},{14,66},{-5,66},{-5,52.4}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(crossHX.Outlet_fl2, sinkP1.flangeB) annotation (Line(
            points={{-5.78,-24.4},{-5.78,-46},{9.6,-46}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(graphics));
      end CrossHx;

      model Pump

        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump
                                                pump(
          PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.UD,
          X_pp0=1,
          eta_em=1,
          eta_is=1,
          epsilon_v=1,
          f_pp0=50,
          V_dot_max=0.0039,
          M_dot_start=3,
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
          PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.FF)
          annotation (Placement(transformation(extent={{-14,-16},{12,10}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceP sourceP(
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
          p0=115794,
          T_0=389.15) annotation (Placement(transformation(extent={{-100,-16},{-80,4}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package                                                               Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66, p0=150000)
          annotation (Placement(transformation(extent={{54,2},{74,22}})));
        Modelica.Blocks.Sources.Step step(
        startTime = 0.5,
        offset= 0.5,
        height=0.1)
         annotation (Placement(transformation(extent={{-90,32},{-70,52}})));
      equation
        connect(sourceP.flange, pump.InFlow) annotation (Line(
            points={{-80.6,-6},{-46,-6},{-46,-2.35},{-10.36,-2.35}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pump.OutFlow, sinkP.flangeB) annotation (Line(
            points={{6.28,6.62},{30.14,6.62},{30.14,12},{55.6,12}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(step.y, pump.flow_in)
          annotation (Line(points={{-69,42},{-36,42},{-36,7.4},{-5.16,7.4}}, color={0,0,127}));
      end Pump;

      model Solar_SolaFieldSchott

       ThermoCycle.Components.Units.Solar.SolarField_SchottSopo         solarCollectorIncSchott(
          Mdotnom=0.5,
          redeclare model FluidHeatTransferModel =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Ideal,
          redeclare
            ThermoCycle.Components.HeatFlow.Walls.SolarAbsorber.Geometry.Schott_SopoNova.Schott_2008_PTR70_Vacuum
            CollectorGeometry,
          redeclare package Medium1 = ThermoCycle.Media.Water,
          Ns=2,
          Tstart_inlet=298.15,
          Tstart_outlet=373.15,
          pstart=1000000)
          annotation (Placement(transformation(extent={{-34,-28},{8,42}})));

       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(Mdot_0=0.5,
          redeclare package Medium = ThermoCycle.Media.Water,
          p=1000000)
          annotation (Placement(transformation(extent={{-66,-70},{-46,-50}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium =
                     ThermoCycle.Media.Water, p0=1000000)
          annotation (Placement(transformation(extent={{22,56},{42,76}})));
        Modelica.Blocks.Sources.Constant const(k=25 + 273.15)
          annotation (Placement(transformation(extent={{-118,-22},{-98,-2}})));
        Modelica.Blocks.Sources.Constant const1(k=0)
          annotation (Placement(transformation(extent={{-118,12},{-98,32}})));
        Modelica.Blocks.Sources.Constant const3(k=0)
          annotation (Placement(transformation(extent={{-98,44},{-78,64}})));
        Modelica.Blocks.Sources.Step step(
          offset=850,
          startTime=100,
          height=0)
          annotation (Placement(transformation(extent={{-110,-54},{-90,-34}})));
      equation
        connect(sourceMdot.flangeB, solarCollectorIncSchott.InFlow) annotation (
            Line(
            points={{-47,-60},{-34,-60},{-34,7}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinkP.flangeB, solarCollectorIncSchott.OutFlow) annotation (Line(
            points={{23.6,66},{7.58,66},{7.58,7.7}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(const3.y, solarCollectorIncSchott.v_wind) annotation (Line(
            points={{-77,54},{-56,54},{-56,34},{2.54,34},{2.54,39.2}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(const1.y, solarCollectorIncSchott.Theta) annotation (Line(
            points={{-97,22},{-48,22},{-48,39.55},{-7.33,39.55}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(const.y, solarCollectorIncSchott.Tamb) annotation (Line(
            points={{-97,-12},{-54,-12},{-54,39.55},{-17.83,39.55}},
            color={0,0,127},
            smooth=Smooth.None));

        connect(step.y, solarCollectorIncSchott.DNI) annotation (Line(
            points={{-89,-44},{-56,-44},{-56,39.2},{-27.28,39.2}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(graphics),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Solar_SolaFieldSchott;

      model Tank_pL

      ThermoCycle.Components.Units.Tanks.Tank_pL tank_pL(
          Vtot=0.01,
          impose_L=true,
          impose_pressure=false,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          pstart=228288)
          annotation (Placement(transformation(extent={{-52,10},{-32,30}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          UseT=false,
          h_0=2.4E5,
          Mdot_0=0.35,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          p=228288) annotation (Placement(transformation(extent={{-76,46},{-56,66}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SinkVdot sinkVdot(Vdot_0=2e-4,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          pstart=200000)
          annotation (Placement(transformation(extent={{-32,-40},{-12,-20}})));
      equation
        connect(sourceWF.flangeB, tank_pL.InFlow)
                                               annotation (Line(
            points={{-57,56},{-42,56},{-42,28.4}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(tank_pL.OutFlow, sinkVdot.flangeB)
                                                annotation (Line(
            points={{-42,11.2},{-42,-30},{-31.8,-30}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{100,
                  100}}), graphics));
      end Tank_pL;

      model OpenTank

       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          h_0=2.4E5,
          redeclare package Medium = ThermoCycle.Media.Water,
          Mdot_0=3,
          UseT=true,
          p=200000,
          T_0=313.15)
                    annotation (Placement(transformation(extent={{-76,46},{-56,66}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SinkVdot sinkVdot(
          redeclare package Medium = ThermoCycle.Media.Water,
          h_out=167691,
          Vdot_0=0.001,
          pstart=200000)
          annotation (Placement(transformation(extent={{46,26},{66,46}})));
        ThermoCycle.Components.Units.Tanks.OpenTank
                                        openTank(
          redeclare package Medium = ThermoCycle.Media.Water,
          H_D=2.5,
          V_tank=4,
          L_lstart=0.3,
          Mdotnom=2,
          p_ext=300000,
          Tstart=373.15)
          annotation (Placement(transformation(extent={{-16,38},{4,58}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp(redeclare package
            Medium = ThermoCycle.Media.Water)
          annotation (Placement(transformation(extent={{12,28},{32,48}})));
      equation
        connect(sourceWF.flangeB, openTank.InFlow) annotation (Line(
            points={{-57,56},{-50,56},{-50,42},{-15.8,42},{-15.8,39.6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp.OutFlow, sinkVdot.flangeB) annotation (Line(
            points={{29,33.2},{37.5,33.2},{37.5,36},{46.2,36}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(openTank.OutFlow, sensTp.InFlow) annotation (Line(
            points={{3.8,39.6},{10,39.6},{10,33.2},{15,33.2}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{
                  100,100}}),
                          graphics),
          experiment(StopTime=100),
          __Dymola_experimentSetupOutput);
      end OpenTank;

      model ExpansionTank

       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          h_0=2.4E5,
          Mdot_0=3,
          UseT=true,
          redeclare package Medium = ThermoCycle.Media.Water,
          p=200000,
          T_0=313.15)
                    annotation (Placement(transformation(extent={{-56,-34},{-36,-14}})));
        ThermoCycle.Components.Units.Tanks.ExpansionTank expansionTank(
          H_D=2.5,
          V_tank=5,
          L_lstart=0.2,
          Mdotnom=2,
          Unom=3,
          redeclare package Medium = ThermoCycle.Media.Water,
          p_const=false,
          Tstart=313.15,
          pstart=200000)
          annotation (Placement(transformation(extent={{6,2},{36,38}})));
         // redeclare model HeatTransfer =
          //    ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Ideal,

       ThermoCycle.Components.FluidFlow.Reservoirs.SinkVdot sinkVdot(
          redeclare package Medium = ThermoCycle.Media.Water,
          h_out=167691,
          Vdot_0=0.001,
          pstart=200000)
          annotation (Placement(transformation(extent={{62,-22},{82,-2}})));
      equation
        connect(sourceWF.flangeB, expansionTank.InFlow) annotation (Line(
            points={{-37,-24},{21,-24},{21,2.36}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(expansionTank.InFlow, sinkVdot.flangeB) annotation (Line(
            points={{21,2.36},{21,-12},{62.2,-12}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{
                  100,100}}),
                          graphics),
          experiment(StopTime=100),
          __Dymola_experimentSetupOutput);
      end ExpansionTank;

      model Compressor

        Modelica.Blocks.Sources.Ramp N_rot(
          duration=100,
          startTime=400,
          offset=48.25,
          height=0)      annotation (Placement(transformation(
              extent={{-5,-5},{5,5}},
              rotation=0,
              origin={-13,77})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
            package Medium =
              CoolProp2Modelica.Test.benchmark.fluids.propane_CP_TTSE,
            p0=2000000)
          annotation (Placement(transformation(extent={{8,-10},{28,10}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          h_0=503925,
          redeclare package Medium =
              CoolProp2Modelica.Test.benchmark.fluids.propane_CP_TTSE,
          UseT=true,
          Mdot_0=0.01,
          T_0=323.15)
          annotation (Placement(transformation(extent={{-92,40},{-72,60}})));
       ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                generator
          annotation (Placement(transformation(extent={{16,28},{36,48}})));
        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor
                                                      compressor(redeclare
            package Medium =
                     CoolProp2Modelica.Test.benchmark.fluids.propane_CP_TTSE,
            T_su_start=373.15)
          annotation (Placement(transformation(extent={{-46,4},{-4,44}})));
      equation
        connect(N_rot.y, generator.f) annotation (Line(
            points={{-7.5,77},{26.4,77},{26.4,47.4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sourceWF.flangeB, compressor.InFlow) annotation (Line(
            points={{-73,50},{-54,50},{-54,48},{-36.9,48},{-36.9,37}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(compressor.OutFlow, sinkPFluid.flangeB) annotation (Line(
            points={{-9.95,17.3333},{-2,17.3333},{-2,0},{9.6,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(generator.shaft, compressor.flange_elc) annotation (Line(
            points={{17.4,38},{6,38},{6,24},{-11,24}},
            color={0,0,0},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-120,-100},{80,100}},
                preserveAspectRatio=false),
                            graphics), Icon(coordinateSystem(extent={{-120,-100},
                  {80,100}})),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Compressor;

      model Compressor_EN12900

        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
            package Medium = ThermoCycle.Media.R407c, p0=2000000)
          annotation (Placement(transformation(extent={{8,-10},{28,10}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          h_0=503925,
          UseT=true,
          Mdot_0=0.05,
          redeclare package Medium = ThermoCycle.Media.R407c,
          T_0=323.15)
          annotation (Placement(transformation(extent={{-96,38},{-76,58}})));
        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor_EN12900
                                                              compressor(redeclare
            package Medium = ThermoCycle.Media.R407c, redeclare function
            CPmodel =
              ThermoCycle.Functions.Compressors_EN12900.ZRD42KCE_TFD)
          annotation (Placement(transformation(extent={{-46,4},{-4,44}})));
      equation
        connect(sourceWF.flangeB, compressor.InFlow) annotation (Line(
            points={{-77,48},{-36.9,48},{-36.9,37}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(compressor.OutFlow, sinkPFluid.flangeB) annotation (Line(
            points={{-9.95,17.3333},{-2,17.3333},{-2,0},{9.6,0}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-120,-100},{80,100}},
                preserveAspectRatio=false),
                            graphics), Icon(coordinateSystem(extent={{-120,-100},
                  {80,100}})),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Compressor_EN12900;

      model valve

        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
            package Medium = ThermoCycle.Media.R245fa_CP,  p0=400000)
          annotation (Placement(transformation(extent={{24,32},{44,52}})));
        Modelica.Blocks.Sources.Ramp ramp(
          duration=2,
          startTime=1,
          height=4E5,
          offset=4E5)
          annotation (Placement(transformation(extent={{-104,78},{-84,98}})));
        Real y;
        ThermoCycle.Components.Units.PdropAndValves.Valve valve(
          UseNom=true,
          Mdot_nom=0.5,
          p_nom=400000,
          T_nom=298.15,
          DELTAp_nom=40000)
          annotation (Placement(transformation(extent={{-38,28},{-18,48}})));
        Modelica.Blocks.Sources.Ramp ramp1(
          duration=2,
          height=-1,
          offset=1,
          startTime=10)
          annotation (Placement(transformation(extent={{-38,74},{-18,94}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid1(
                                                                     redeclare
            package Medium = ThermoCycle.Media.R245fa_CP,  p0=400000)
          annotation (Placement(transformation(extent={{-66,30},{-86,50}})));
      equation
        y = ThermoCycle.Functions.weightingfactor(
              t_init=2,
              length=5,
              t=time)
        annotation (Diagram(graphics), uses(Modelica(version="3.2")));
        connect(valve.OutFlow, sinkPFluid.flangeB) annotation (Line(
            points={{-19,38},{6,38},{6,42},{25.6,42}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp1.y, valve.cmd) annotation (Line(
            points={{-17,84},{4,84},{4,62},{-28,62},{-28,46}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sinkPFluid1.flangeB, valve.InFlow) annotation (Line(
            points={{-67.6,40},{-52,40},{-52,38},{-37,38}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, sinkPFluid1.in_p0) annotation (Line(
            points={{-83,88},{-72,88},{-72,48.8}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                            graphics),
          experiment(StopTime=50),
          __Dymola_experimentSetupOutput);
      end valve;

      model Test_water
      replaceable package Medium = CoolProp2Modelica.Media.WaterIF95_FP;

      parameter Modelica.SIunits.Temperature T_su= 30 + 273.15;
      parameter Modelica.SIunits.Temperature T_ex= 35 + 273.15;
      parameter Modelica.SIunits.Pressure p = 1e5;

      Modelica.SIunits.Density rho_su;
      Modelica.SIunits.Density rho_ex;

      equation
      rho_su = Medium.density_pT(p,T_su);

      rho_ex = Medium.density_pT(p,T_ex);

      end Test_water;

      model Test_Properties
      replaceable package Medium = ThermoCycle.Media.R245fa_CP;

      parameter Modelica.SIunits.Pressure P = 211000 "pressure in Pa";

      parameter Modelica.SIunits.SpecificEnthalpy h = 245e3 "Enthalpy";

      Modelica.SIunits.Temperature T;
      equation

        T = Medium.temperature_ph(P,h);

      end Test_Properties;

      model HeatTransfer
        extends Modelica.Icons.Example;

      model InputSelector
      replaceable package Medium = ThermoCycle.Media.DummyFluid constrainedby
            Modelica.Media.Interfaces.PartialTwoPhaseMedium "Medium"
          annotation(choicesAllMatching=true);

      // Settings for heat transfer
      Medium.ThermodynamicState state(phase(start=0));
      // Settings for correlation
      parameter Modelica.SIunits.MassFlowRate m_dot_nom = 1 "Nomnial Mass flow rate"
                                 annotation (Dialog(tab="Heat transfer"));
      parameter Modelica.SIunits.CoefficientOfHeatTransfer U_nom_l = 2000
            "Nominal heat transfer coefficient liquid side"
                                                        annotation (Dialog(tab="Heat transfer"));
      parameter Modelica.SIunits.CoefficientOfHeatTransfer U_nom_tp = 3000
            "Nominal heat transfer coefficient two phase side"
                                                           annotation (Dialog(tab="Heat transfer"));
      parameter Modelica.SIunits.CoefficientOfHeatTransfer U_nom_v = 1500
            "Nominal heat transfer coefficient vapor side"
                                                       annotation (Dialog(tab="Heat transfer"));
      Medium.AbsolutePressure p;
      Medium.SpecificEnthalpy h;
      Medium.SpecificEnthalpy h_start;
      Medium.SpecificEnthalpy h_end;
      Modelica.SIunits.MassFlowRate m_dot "Inlet massflow";
      Real x "Vapor quality";
      Real y "Relative position";
      Modelica.SIunits.Time c = 10;

      Medium.ThermodynamicState bubbleState(h(start=0));
          Medium.ThermodynamicState dewState(h(start=0));

      replaceable model HeatTransfer =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.MassFlowDependence
        constrainedby
            ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.BaseClasses.PartialConvectiveCorrelation
            "Heat transfer model"
          annotation(choicesAllMatching=true);

          HeatTransfer heatTransfer(
        redeclare final package Medium = Medium,
        final n = 1,
        FluidState = {state},
        Mdotnom = m_dot_nom,
        Unom_l =  U_nom_l,
        Unom_tp = U_nom_tp,
        Unom_v =  U_nom_v,
        M_dot = m_dot,
        x = x);

          parameter Medium.AbsolutePressure p_start = 1e5 "Start pressure";
          parameter Medium.AbsolutePressure p_end = p_start "Final pressure";

          parameter Modelica.SIunits.MassFlowRate m_dot_start = 1 "Start flow rate";
          parameter Modelica.SIunits.MassFlowRate m_dot_end = m_dot_start
            "Final flow rate";

          parameter Boolean twoPhase = false "is two-phase medium?";
          parameter Medium.SpecificEnthalpy h_start_in = 0 "Start enthalpy"
            annotation(Dialog(enable = not twoPhase));

          parameter Medium.SpecificEnthalpy h_end_in = h_start_in "Final enthalpy"
            annotation(Dialog(enable = not twoPhase));

        //parameter Discretizations Discretization=ThermoCycle.Functions.Enumerations.Discretizations.centr_diff
        //  "Selection of the spatial discretization scheme"  annotation (Dialog(tab="Numerical options"));

      // Settings for heat transfer
      // replaceable package Medium=Modelica.Media.Interfaces.PartialMedium
      // input Medium.ThermodynamicState[n] FluidState
      // Settings for correlation
      // input Modelica.SIunits.MassFlowRate Mdotnom "Nomnial Mass flow rate";
      // input Modelica.SIunits.CoefficientOfHeatTransfer Unom_l
      //     "Nominal heat transfer coefficient liquid side";
      // input Modelica.SIunits.CoefficientOfHeatTransfer Unom_tp
      //     "nominal heat transfer coefficient two phase side";
      // input Modelica.SIunits.CoefficientOfHeatTransfer Unom_v
      //     "nominal heat transfer coefficient vapor side";
      // input Modelica.SIunits.MassFlowRate M_dot "Inlet massflow";
      // input Modelica.SIunits.QualityFactor x "Vapor quality";

      equation
        if twoPhase then
          bubbleState = Medium.setBubbleState(Medium.setSat_p(Medium.pressure(state)));
          dewState    = Medium.setDewState(   Medium.setSat_p(Medium.pressure(state)));
          x           = (Medium.specificEnthalpy(state) - Medium.specificEnthalpy(bubbleState))/(Medium.specificEnthalpy(dewState) - Medium.specificEnthalpy(bubbleState));
          h_start     = Medium.specificEnthalpy(bubbleState) - 0.25*(Medium.specificEnthalpy(dewState) - Medium.specificEnthalpy(bubbleState));
          h_end       = Medium.specificEnthalpy(dewState)    + 0.25*(Medium.specificEnthalpy(dewState) - Medium.specificEnthalpy(bubbleState));
        else
          bubbleState = state;
          dewState    = state;
          x           = 0;
          h_start     = h_start_in;
          h_end       = h_end_in;
        end if;
        y = time/c;
        p     = (1-y) * p_start     + y * p_end;
        m_dot = (1-y) * m_dot_start + y * m_dot_end;
        h     = (1-y) * h_start     + y * h_end;
        state = Medium.setState_phX(p=p,h=h);

      end InputSelector;

        InputSelector tester(
          h_start_in=100e3,
          twoPhase=true,
          redeclare package Medium = ThermoCycle.Media.R134a_CP(substanceNames={"R134a|debug=0|calc_transport=1|enable_EXTTP=1|enable_TTSE=0"}),
          m_dot_start=3,
          m_dot_nom=3,
          redeclare model HeatTransfer =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Smoothed,
          p_start=500000)
          annotation (Placement(transformation(extent={{-42,42},{-22,62}})));

        annotation (experiment(StopTime=10));
      end HeatTransfer;

      model TestNozzle

        ThermoCycle.Components.Units.PdropAndValves.Nozzle
          leakageNozzle(
          redeclare package Medium = CoolProp2Modelica.Media.R718_CP,
          Afull=5e-5,
          gamma_start=1.3,
          Use_gamma=false,
          P_su_start=1500000,
          T_su_start=623.15,
          P_ex_start=1400000)
          annotation (Placement(transformation(extent={{-10,-14},{10,6}})));
        Modelica.Blocks.Sources.Sine sine(
          freqHz=0.25,
          phase=0,
          amplitude=4.1e5,
          offset=12e5)
          annotation (Placement(transformation(extent={{-92,36},{-72,56}})));
        Modelica.Fluid.Sources.Boundary_ph boundary1(
          nPorts=1,
          use_p_in=true,
          redeclare package Medium = CoolProp2Modelica.Media.R718_CP)
          annotation (Placement(transformation(extent={{80,-14},{60,6}})));
        Modelica.Fluid.Sources.Boundary_pT boundary(
          nPorts=1,
          redeclare package Medium = CoolProp2Modelica.Media.R718_CP,
          p=1500000,
          T=623.15)
          annotation (Placement(transformation(extent={{-80,-14},{-60,6}})));
      equation
        connect(sine.y,boundary1. p_in) annotation (Line(
            points={{-71,46},{88,46},{88,4},{82,4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(boundary.ports[1],leakageNozzle. su) annotation (Line(
            points={{-60,-4},{-36,-4},{-36,-3.8},{-10.2,-3.8}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(leakageNozzle.ex,boundary1. ports[1]) annotation (Line(
            points={{10,-3.8},{36,-3.8},{36,-4},{60,-4}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=10),
          __Dymola_experimentSetupOutput);
      end TestNozzle;

      model Solar_SolaFieldSoltigua

      ThermoCycle.Components.Units.Solar.SolarField_Soltigua_Inc        solarCollectorIncSchott(
          Mdotnom=0.5,
          Ns=2,
          redeclare
            ThermoCycle.Components.HeatFlow.Walls.SolarAbsorber.Geometry.Soltigua.PTMx_18
            CollectorGeometry,
          redeclare model FluidHeatTransferModel =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
          redeclare package Medium1 =
              ThermoCycle.Media.Incompressible.IncompressibleTables.TherminolSP,
          Tstart_inlet=298.15,
          Tstart_outlet=373.15,
          pstart=1000000)
          annotation (Placement(transformation(extent={{-34,-28},{8,42}})));

       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(Mdot_0=0.5,
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.TherminolSP,
          p=200000)
          annotation (Placement(transformation(extent={{-66,-70},{-46,-50}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.TherminolSP, p0=
              200000)
          annotation (Placement(transformation(extent={{22,56},{42,76}})));
        Modelica.Blocks.Sources.Constant const(k=25 + 273.15)
          annotation (Placement(transformation(extent={{-118,-22},{-98,-2}})));
        Modelica.Blocks.Sources.Constant const1(k=0)
          annotation (Placement(transformation(extent={{-118,12},{-98,32}})));
        Modelica.Blocks.Sources.Constant const3(k=0)
          annotation (Placement(transformation(extent={{-98,44},{-78,64}})));
        Modelica.Blocks.Sources.Step step(
          startTime=100,
          height=0,
          offset=450)
          annotation (Placement(transformation(extent={{-110,-54},{-90,-34}})));
        Modelica.Blocks.Sources.BooleanConstant booleanConstant
          annotation (Placement(transformation(extent={{-162,-74},{-142,-54}})));
      equation
        connect(sourceMdot.flangeB, solarCollectorIncSchott.InFlow) annotation (
            Line(
            points={{-47,-60},{-8.8,-60},{-8.8,-28.6364}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinkP.flangeB, solarCollectorIncSchott.OutFlow) annotation (Line(
            points={{23.6,66},{-6,66},{-6,41.3636}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(const3.y, solarCollectorIncSchott.v_wind) annotation (Line(
            points={{-77,54},{-56,54},{-56,34},{-31.2,34},{-31.2,36.2727}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(const1.y, solarCollectorIncSchott.Theta) annotation (Line(
            points={{-97,22},{-48,22},{-48,22.9091},{-31.6667,22.9091}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(const.y, solarCollectorIncSchott.Tamb) annotation (Line(
            points={{-97,-12},{-54,-12},{-54,8.90909},{-31.6667,8.90909}},
            color={0,0,127},
            smooth=Smooth.None));

        connect(step.y, solarCollectorIncSchott.DNI) annotation (Line(
            points={{-89,-44},{-56,-44},{-56,-4.13636},{-31.9,-4.13636}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(booleanConstant.y, solarCollectorIncSchott.Defocusing) annotation (
            Line(
            points={{-141,-64},{-110,-64},{-110,-68},{-68,-68},{-68,-17.8182},{
                -31.6667,-17.8182}},
            color={255,0,255},
            smooth=Smooth.None));
        annotation (Diagram(graphics),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Solar_SolaFieldSoltigua;

      model Test_BoilerSystem
        ThermoCycle.Components.Units.HeatExchangers.BoilerSystem boilerSystem(
          Mdot_w_nom=2.9,
          pinch_start=25,
          DELTAT_approach=1,
          x_ex_ev_nom=0.307,
          p_start=3635000,
          T_w_su_start=380.27,
          T_w_ex_start=627,
          T_w_su_SH2_start=593.5,
          T_sf_su_start=653.15)
          annotation (Placement(transformation(extent={{-12,-28},{48,36}})));
        Queralt.Components.Valve_lin valve_water_vs_sh2(
          redeclare package Medium = ThermoCycle.Media.Water,
          UseNom=true,
          CheckValve=true,
          Xopen=0,
          Mdot_nom=0.25,
          DELTAp_nom=50000) annotation (Placement(transformation(
              extent={{-7,7},{7,-7}},
              rotation=180,
              origin={65,27})));
        Queralt.Components.Valve_lin valve_feed_water(
          redeclare package Medium = ThermoCycle.Media.Water,
          UseNom=true,
          Xopen=1,
          Mdot_nom=2.8429767731178850E+000,
          CheckValve=true,
          DELTAp_nom=50000) annotation (Placement(transformation(
              extent={{-8,-8},{8,8}},
              rotation=90,
              origin={32,-36})));
        Queralt.Components.Valve_lin valve_solar(
          UseNom=true,
          Xopen=1,
          CheckValve=true,
          Mdot_nom=25,
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.FlueGas,
          DELTAp_nom=10000)
          annotation (Placement(transformation(extent={{-28,76},{-12,92}})));
        Queralt.Components.FlangeConverter flangeConverter(redeclare package
            Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.FlueGas)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={4,64})));
        Queralt.Components.SinkP_pT sinkP1(redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.FlueGas, p0=
              2000000)
          annotation (Placement(transformation(extent={{-34,-44},{-48,-30}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
          redeclare package Medium = ThermoCycle.Media.Water,
          Mdot_0=2.8429767731178850E+000,
          T_0=333.15)
          annotation (Placement(transformation(extent={{74,-50},{94,-30}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium = ThermoCycle.Media.Water, p0=3430000)
          annotation (Placement(transformation(extent={{58,68},{78,88}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          Mdot_0=25,
          redeclare package Medium = Queralt.FlueGas,
          T_0=673.15)
          annotation (Placement(transformation(extent={{-70,76},{-50,96}})));
      equation
        connect(boilerSystem.flangeA1, valve_water_vs_sh2.OutFlow) annotation (Line(
            points={{40.5,20.32},{49.25,20.32},{49.25,27},{58.7,27}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(valve_water_vs_sh2.InFlow, boilerSystem.flangeA) annotation (Line(
            points={{71.3,27},{82,27},{82,-25.12},{35.7,-25.12}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(boilerSystem.flangeA, valve_feed_water.OutFlow) annotation (Line(
            points={{35.7,-25.12},{35.7,-24},{32,-24},{32,-28.8}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(valve_solar.OutFlow,flangeConverter. flange_ph) annotation (Line(
            points={{-12.8,84},{3.9,84},{3.9,69.1}},
            color={0,127,0},
            smooth=Smooth.None,
            pattern=LinePattern.Dash,
            thickness=0.5));
        connect(flangeConverter.flange_pT, boilerSystem.inlet_sf) annotation (Line(
            points={{3.9,59.1},{3.9,47.55},{3,47.55},{3,34.72}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(sinkP1.flangeB, boilerSystem.outlet_sf) annotation (Line(
            points={{-35.12,-37},{3,-37},{3,-26.4}},
            color={0,127,0},
            smooth=Smooth.None));
        connect(sinkP.flangeB, boilerSystem.flangeB) annotation (Line(
            points={{59.6,78},{36,78},{36,74},{24.3,74},{24.3,34.4}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sourceMdot.flangeB, valve_feed_water.InFlow) annotation (Line(
            points={{93,-40},{94,-40},{94,-66},{32,-66},{32,-43.2}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sourceMdot1.flangeB, valve_solar.InFlow) annotation (Line(
            points={{-51,86},{-36,86},{-36,84},{-27.2,84}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end Test_BoilerSystem;

      model Test_Cell1D

        ThermoCycle.Components.FluidFlow.Pipes.Cell1Dim flow1Dim(
          Ai=0.2,
          Unom_l=400,
          Unom_tp=1000,
          Unom_v=400,
          max_drhodt=50,
          Vi=0.005,
          redeclare package Medium = ThermoCycle.Media.R407c_CP,
          Mdotnom=0.3335,
          hstart=84867,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind,

          pstart=866735,
          redeclare model HeatTransfer =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.MassFlowDependence)
          annotation (Placement(transformation(extent={{-28,10},{-8,30}})));

        ThermoCycle.Components.HeatFlow.Sources.Source_T_cell source_T
          annotation (Placement(transformation(extent={{-32,50},{-12,70}})));
        Modelica.Blocks.Sources.Constant const(k=273.15 + 140)
          annotation (Placement(transformation(extent={{-66,72},{-46,92}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          redeclare package Medium = ThermoCycle.Media.R407c_CP,
          UseT=false,
          h_0=84867,
          Mdot_0=0.3334,
          p=888343,
          T_0=356.26)
          annotation (Placement(transformation(extent={{-84,10},{-64,30}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(
          redeclare package Medium = ThermoCycle.Media.R407c_CP,
          h=254381,
          p0=866735)
          annotation (Placement(transformation(extent={{40,4},{60,24}})));
      equation
        connect(const.y, source_T.Temperature) annotation (Line(
            points={{-45,82},{-28,82},{-28,80},{-21,80},{-21,65}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(flow1Dim.Wall_int, source_T.ThermalPortCell) annotation (Line(
            points={{-18,25},{-18,56.9},{-21.1,56.9}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(sourceMdot1.flangeB, flow1Dim.InFlow) annotation (Line(
            points={{-65,20},{-28,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, sinkP.flangeB) annotation (Line(
            points={{-8,20.1},{16,20.1},{16,16},{41.6,16},{41.6,14}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                  100,100}}), graphics),
          experiment(StopTime=50),
          __Dymola_experimentSetupOutput);
      end Test_Cell1D;

      model Test_Compressor

        Modelica.Blocks.Sources.Ramp N_rot(
          duration=100,
          startTime=400,
          offset=48.25,
          height=0)      annotation (Placement(transformation(
              extent={{-5,-5},{5,5}},
              rotation=0,
              origin={-13,77})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
            package Medium = ThermoCycle.Media.Propane_CP, p0=2000000)
          annotation (Placement(transformation(extent={{8,-10},{28,10}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          h_0=503925,
          UseT=true,
          Mdot_0=0.01,
          redeclare package Medium = ThermoCycle.Media.Propane_CP,
          T_0=323.15)
          annotation (Placement(transformation(extent={{-92,40},{-72,60}})));
       ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                generator
          annotation (Placement(transformation(extent={{16,28},{36,48}})));
        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor
                                                      compressor(redeclare
            package Medium =
                     ThermoCycle.Media.Propane_CP, T_su_start=373.15)
          annotation (Placement(transformation(extent={{-46,4},{-4,44}})));
      equation
        connect(N_rot.y, generator.f) annotation (Line(
            points={{-7.5,77},{26.4,77},{26.4,47.4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sourceWF.flangeB, compressor.InFlow) annotation (Line(
            points={{-73,50},{-54,50},{-54,48},{-36.9,48},{-36.9,37}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(compressor.OutFlow, sinkPFluid.flangeB) annotation (Line(
            points={{-9.95,17.3333},{-2,17.3333},{-2,0},{9.6,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(generator.shaft, compressor.flange_elc) annotation (Line(
            points={{17.4,38},{6,38},{6,24},{-11,24}},
            color={0,0,0},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-120,-100},{80,100}},
                preserveAspectRatio=false),
                            graphics), Icon(coordinateSystem(extent={{-120,-100},
                  {80,100}})),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Test_Compressor;

      model Test_Compressor_EN12900

        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
            package Medium = ThermoCycle.Media.R407c_CP, p0=2000000)
          annotation (Placement(transformation(extent={{8,-10},{28,10}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          h_0=503925,
          UseT=true,
          Mdot_0=0.05,
          redeclare package Medium = ThermoCycle.Media.R407c_CP,
          T_0=323.15)
          annotation (Placement(transformation(extent={{-96,38},{-76,58}})));
        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor_EN12900
                                                              compressor(redeclare
            package Medium = ThermoCycle.Media.R407c_CP, redeclare function
            CPmodel =
              ThermoCycle.Functions.Compressors_EN12900.ZRD42KCE_TFD)
          annotation (Placement(transformation(extent={{-46,4},{-4,44}})));
      equation
        connect(sourceWF.flangeB, compressor.InFlow) annotation (Line(
            points={{-77,48},{-36.9,48},{-36.9,37}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(compressor.OutFlow, sinkPFluid.flangeB) annotation (Line(
            points={{-9.95,17.3333},{-2,17.3333},{-2,0},{9.6,0}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-120,-100},{80,100}},
                preserveAspectRatio=false),
                            graphics), Icon(coordinateSystem(extent={{-120,-100},
                  {80,100}})),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Test_Compressor_EN12900;

      model Test_CrossCondenser

       ThermoCycle.Components.Units.HeatExchangers.CrossCondenser condenser(
          redeclare package Medium1 = Media.WaterIF95_FP,
          N=5,
          c_wall=500,
          steadystate_T_wall=false,
          redeclare package Medium2 =
              Modelica.Media.Incompressible.Examples.Glycol47,
          Discretization_sf=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
          V_sf=1.5,
          A_wf=279.61,
          A_sf=252.09,
          M_wall_tot=2552.13,
          redeclare model Medium2HeatTransferModel =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.MassFlowDependence,
          Mdotnom_sf=585176/3600,
          U_wf=9250,
          h_su_wf_start=2.053e6,
          h_ex_wf_start=2.36e5,
          Unom_sf=769,
          Tstart_sf_in=285.84,
          Tstart_sf_out=295.1,
          T_sat_start=329.49,
          T_start_wall=326.65)
          annotation (Placement(transformation(extent={{-24,6},{8,32}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceSF(
          UseT=true,
          redeclare package Medium =
              Modelica.Media.Incompressible.Examples.Glycol47,
          Mdot_0=585176/3600,
          p=248000,
          T_0=283.15)
          annotation (Placement(transformation(extent={{-66,-2},{-50,14}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkSF(redeclare
            package Medium = Modelica.Media.Incompressible.Examples.Glycol47,
            p0=248000)
          annotation (Placement(transformation(extent={{-54,22},{-66,34}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot source_wf(
          UseT=true,
          redeclare package Medium = Media.WaterIF95_FP,
          Mdot_0=585176/3600,
          p=248000,
          T_0=283.15)
          annotation (Placement(transformation(extent={{-14,52},{2,68}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sink_wf(redeclare
            package Medium = Media.WaterIF95_FP, p0=248000)
          annotation (Placement(transformation(extent={{-18,-24},{-30,-12}})));
      equation

        connect(sourceSF.flangeB, condenser.InFlow_fl2) annotation (Line(
            points={{-50.8,6},{-36,6},{-36,0},{-20.8,0},{-20.8,12.5}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinkSF.flangeB, condenser.OutFlow_fl2) annotation (Line(
            points={{-54.96,28},{-46,28},{-46,40},{-20.8,40},{-20.8,25.5}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(source_wf.flangeB, condenser.InFlow_fl1) annotation (Line(
            points={{1.2,60},{10,60},{10,38},{-4.8,38},{-4.8,28.1}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sink_wf.flangeB, condenser.OutFlow_fl1) annotation (Line(
            points={{-18.96,-18},{-14,-18},{-14,-16},{-4.8,-16},{-4.8,9.9}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                            graphics),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Test_CrossCondenser;

      model Test_CrossHx

        ThermoCycle.Components.Units.HeatExchangers.CrossHX crossHX(
          V_wf=1.36e-4,
          A_wf=0.06248,
          V_sf=0,
          A_sf=1.091,
          Mdotnom_wf=0.19/12,
          Mdotnom_sf=6/12/2,
          M_wall_tot=2.85,
          c_wall=660,
          Unom_l=430,
          Unom_tp=4400,
          Unom_v=660,
          Unom_sf=50,
          UseNom_sf=true,
          T_nom_sf=298.15,
          DELTAp_quad_nom_sf=50000,
          pstart_wf=211000,
          Tstart_wf_in=323.15)
          annotation (Placement(transformation(extent={{-44,-26},{34,54}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceR245faCool(
          Mdot_0=0.19/12,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          T_0=323.15)
          annotation (Placement(transformation(extent={{-90,4},{-70,24}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceAir(
          redeclare package Medium = Modelica.Media.Air.SimpleAir,
          Mdot_0=6/12/2,
          T_0=303.81)
          annotation (Placement(transformation(extent={{-10,76},{10,96}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium = ThermoCycle.Media.R245fa_CP, p0=211000)
          annotation (Placement(transformation(extent={{82,12},{102,32}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
            package Medium = Modelica.Media.Air.SimpleAir)
          annotation (Placement(transformation(extent={{8,-56},{28,-36}})));
      equation
        connect(sourceR245faCool.flangeB, crossHX.Inlet_fl1) annotation (Line(
            points={{-71,14},{-44.78,14}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(crossHX.Outlet_fl1, sinkP.flangeB) annotation (Line(
            points={{33.22,14},{58.61,14},{58.61,22},{83.6,22}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sourceAir.flangeB, crossHX.Inlet_fl2) annotation (Line(
            points={{9,86},{14,86},{14,66},{-5,66},{-5,52.4}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(crossHX.Outlet_fl2, sinkP1.flangeB) annotation (Line(
            points={{-5.78,-24.4},{-5.78,-46},{9.6,-46}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(graphics));
      end Test_CrossHx;

      model Test_DP

      ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          Mdot_0=-0.5,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          p=400000,
          T_0=298.15)
          annotation (Placement(transformation(extent={{-84,30},{-64,50}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
            package Medium = ThermoCycle.Media.R245fa_CP, p0=400000)
          annotation (Placement(transformation(extent={{24,32},{44,52}})));
        ThermoCycle.Components.Units.PdropAndValves.DP dP(
          h=3,
          UseHomotopy=true,
          A=4e-5,
          constinit=true,
          Mdot_nom=0.5,
          UseNom=true,
          t_init=0.5,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          p_nom=400000,
          T_nom=298.15,
          DELTAp_stat_nom=40000,
          DELTAp_lin_nom=5000,
          DELTAp_quad_nom=60000)
          annotation (Placement(transformation(extent={{-30,32},{-10,52}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=1,
          duration=2,
          startTime=1,
          offset=0.5)
          annotation (Placement(transformation(extent={{-104,78},{-84,98}})));
        Real y;
        ThermoCycle.Components.FluidFlow.Sensors.SensTpSat sensTp(redeclare
            package Medium = ThermoCycle.Media.R245fa_CP)
          annotation (Placement(transformation(extent={{-56,36},{-36,56}})));
        ThermoCycle.Components.FluidFlow.Sensors.SensTpSat sensTp1(redeclare
            package Medium = ThermoCycle.Media.R245fa_CP)
          annotation (Placement(transformation(extent={{-4,36},{16,56}})));
      equation
        connect(ramp.y, sourceWF.in_Mdot) annotation (Line(
            points={{-83,88},{-60,88},{-60,58},{-80,58},{-80,46}},
            color={0,0,127},
            smooth=Smooth.None));
        y = ThermoCycle.Functions.weightingfactor(
              t_init=2,
              length=5,
              t=time)
        annotation (Diagram(graphics), uses(Modelica(version="3.2")));
        connect(sourceWF.flangeB, sensTp.InFlow) annotation (Line(
            points={{-65,40},{-60,40},{-60,41.2},{-53,41.2}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp.OutFlow, dP.InFlow) annotation (Line(
            points={{-39,41.2},{-32,41.2},{-32,42},{-29,42}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(dP.OutFlow, sensTp1.InFlow) annotation (Line(
            points={{-11,42},{-6,42},{-6,41.2},{-1,41.2}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp1.OutFlow, sinkPFluid.flangeB) annotation (Line(
            points={{13,41.2},{20,41.2},{20,42},{25.6,42}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                            graphics),
          experiment(StopTime=50),
          __Dymola_experimentSetupOutput);
      end Test_DP;

      model Test_Expander

        Modelica.Blocks.Sources.Ramp N_rot(
          duration=100,
          startTime=400,
          offset=48.25,
          height=0)      annotation (Placement(transformation(
              extent={{-5,-5},{5,5}},
              rotation=0,
              origin={-13,77})));
       ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Expander
                                                               expander(redeclare
            package Medium = ThermoCycle.Media.R245fa_CP,
                                               V_s=1)
          annotation (Placement(transformation(extent={{-32,16},{-12,36}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
            package Medium = ThermoCycle.Media.R245fa_CP, p0=153400)
          annotation (Placement(transformation(extent={{8,-10},{28,10}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          Mdot_0=0.2588,
          UseT=false,
          h_0=503925,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP)
          annotation (Placement(transformation(extent={{-62,40},{-42,60}})));
       ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                generator
          annotation (Placement(transformation(extent={{16,28},{36,48}})));
      equation
        connect(sourceWF.flangeB, expander.InFlow) annotation (Line(
            points={{-43,50},{-32,50},{-32,29.8333},{-27.6667,29.8333}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(expander.OutFlow, sinkPFluid.flangeB) annotation (Line(
            points={{-14.5,21},{-6,21},{-6,0},{9.6,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(expander.flange_elc, generator.shaft) annotation (Line(
            points={{-15.3333,26.8333},{8,26.8333},{8,38},{17.4,38}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(N_rot.y, generator.f) annotation (Line(
            points={{-7.5,77},{26.4,77},{26.4,47.4}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-80,-40},{80,100}},
                preserveAspectRatio=true),
                            graphics), Icon(coordinateSystem(extent={{-80,-40},{80,
                  100}})),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Test_Expander;

      model Test_ExpansionTank

       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          h_0=2.4E5,
          Mdot_0=3,
          UseT=true,
          redeclare package Medium = ThermoCycle.Media.Water,
          p=200000,
          T_0=313.15)
                    annotation (Placement(transformation(extent={{-56,-34},{-36,-14}})));
        ThermoCycle.Components.Units.Tanks.ExpansionTank expansionTank(
          H_D=2.5,
          V_tank=5,
          L_lstart=0.2,
          Mdotnom=2,
          Unom=3,
          redeclare package Medium = ThermoCycle.Media.Water,
          p_const=false,
          Tstart=313.15,
          pstart=200000)
          annotation (Placement(transformation(extent={{6,2},{36,38}})));
         // redeclare model HeatTransfer =
          //    ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Ideal,

       ThermoCycle.Components.FluidFlow.Reservoirs.SinkVdot sinkVdot(
          redeclare package Medium = ThermoCycle.Media.Water,
          h_out=167691,
          Vdot_0=0.001,
          pstart=200000)
          annotation (Placement(transformation(extent={{62,-22},{82,-2}})));
      equation
        connect(sourceWF.flangeB, expansionTank.InFlow) annotation (Line(
            points={{-37,-24},{21,-24},{21,2.36}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(expansionTank.InFlow, sinkVdot.flangeB) annotation (Line(
            points={{21,2.36},{21,-12},{62.2,-12}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{
                  100,100}}),
                          graphics),
          experiment(StopTime=100),
          __Dymola_experimentSetupOutput);
      end Test_ExpansionTank;

      model Test_Flow1D
        parameter Integer N = 10;
        ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim flow1Dim(
          A=2,
          Unom_l=400,
          Unom_v=400,
          N=N,
          V=0.003,
          Mdotnom=0.3,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
          redeclare package Medium = ThermoCycle.Media.SES36_CP,
          redeclare model Flow1DimHeatTransferModel =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
          Unom_tp=400,
          pstart=1000000,
          Tstart_inlet=323.15,
          Tstart_outlet=373.15)
          annotation (Placement(transformation(extent={{-40,-4},{-2,34}})));

        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-30,42},{4,64}})));
        Modelica.Blocks.Sources.Constant const(k=273.15 + 140)
          annotation (Placement(transformation(extent={{-62,64},{-54,72}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          redeclare package Medium = ThermoCycle.Media.SES36_CP,
          Mdot_0=0.3335,
          UseT=false,
          h_0=84867,
          p=888343,
          T_0=356.26)
          annotation (Placement(transformation(extent={{-102,6},{-76,32}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(
          redeclare package Medium = ThermoCycle.Media.SES36_CP,
          h=254381,
          p0=866735)
          annotation (Placement(transformation(extent={{50,14},{70,34}})));
        Modelica.Blocks.Sources.Sine sine(
          startTime=10,
          offset=0.3335,
          amplitude=0.5,
          phase=0,
          freqHz=0.1)
          annotation (Placement(transformation(extent={{-116,50},{-102,64}})));
        ThermoCycle.Components.FluidFlow.Sensors.SensTpSat T_su_Sensor(
            redeclare package Medium = ThermoCycle.Media.SES36_CP)
          annotation (Placement(transformation(extent={{-66,10},{-46,30}})));
        ThermoCycle.Components.FluidFlow.Sensors.SensTpSat T_ex_Sensor(
            redeclare package Medium = ThermoCycle.Media.SES36_CP)
          annotation (Placement(transformation(extent={{14,14},{34,34}})));
      equation
        connect(source_T.thermalPort, flow1Dim.Wall_int) annotation (Line(
            points={{-13.17,48.49},{-13.17,43.95},{-21,43.95},{-21,22.9167}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(const.y, source_T.Temperature) annotation (Line(
            points={{-53.6,68},{-13,68},{-13,57.4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sine.y, sourceMdot1.in_Mdot) annotation (Line(
            points={{-101.3,57},{-98.5,57},{-98.5,26.8},{-96.8,26.8}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, T_ex_Sensor.InFlow) annotation (Line(
            points={{-5.16667,15.1583},{4,15.1583},{4,19.2},{17,19.2}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(T_ex_Sensor.OutFlow, sinkP.flangeB) annotation (Line(
            points={{31,19.2},{38,19.2},{38,24},{51.6,24}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sourceMdot1.flangeB, T_su_Sensor.InFlow) annotation (Line(
            points={{-77.3,19},{-70,19},{-70,15.2},{-63,15.2}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(T_su_Sensor.OutFlow, flow1Dim.InFlow) annotation (Line(
            points={{-49,15.2},{-43.5,15.2},{-43.5,15},{-36.8333,15}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{80,
                  100}}),     graphics={Text(
                extent={{-62,56},{-26,50}},
                lineColor={0,0,0},
                textString="Thermal port")}),
          experiment(StopTime=50),
          __Dymola_experimentSetupOutput,
          Icon(coordinateSystem(extent={{-120,-100},{80,100}})));
      end Test_Flow1D;

      model Test_Flow1D_MD
        parameter Integer N = 10;
      ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim_MD  flow1Dim(
          A=2,
          U_nom = 400,
          N=N,
          V=0.003,
          Mdotnom=0.3,
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          pstart=1000000,
          Tstart_inlet=323.15,
          Tstart_outlet=373.15)
          annotation (Placement(transformation(extent={{-32,2},{6,40}})));

        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdot_0=0.3335,
          UseT=false,
          h_0=84867,
          p=888343,
          T_0=356.26)
          annotation (Placement(transformation(extent={{-92,6},{-66,32}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          h=254381,
          p0=866735)
          annotation (Placement(transformation(extent={{50,14},{70,34}})));
        Modelica.Blocks.Sources.Sine sine(
          startTime=10,
          offset=0.3335,
          amplitude=0.5,
          phase=0,
          freqHz=0.1)
          annotation (Placement(transformation(extent={{-116,50},{-102,64}})));
        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-20,52},{14,74}})));
        Modelica.Blocks.Sources.Constant const(k=273.15 + 140)
          annotation (Placement(transformation(extent={{-52,74},{-44,82}})));
        ThermoCycle.Components.FluidFlow.Sensors.SensTpSat T_su_Sensor(
            redeclare package Medium = CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{-60,12},{-40,32}})));
        ThermoCycle.Components.FluidFlow.Sensors.SensTpSat T_ex_Sensor(
            redeclare package Medium = CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{18,16},{38,36}})));
      equation
        connect(sine.y, sourceMdot1.in_Mdot) annotation (Line(
            points={{-101.3,57},{-86.8,57},{-86.8,26.8}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(const.y,source_T. Temperature) annotation (Line(
            points={{-43.6,78},{-3,78},{-3,67.4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(source_T.thermalPort, flow1Dim.Wall_int) annotation (Line(
            points={{-3.17,58.49},{-3.17,48},{-16,48},{-16,28.9167},{-13,28.9167}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(sourceMdot1.flangeB, T_su_Sensor.InFlow) annotation (Line(
            points={{-67.3,19},{-59.65,19},{-59.65,17.2},{-57,17.2}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(T_su_Sensor.OutFlow, flow1Dim.InFlow) annotation (Line(
            points={{-43,17.2},{-37.5,17.2},{-37.5,21},{-28.8333,21}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, T_ex_Sensor.InFlow) annotation (Line(
            points={{2.83333,21.1583},{12.4167,21.1583},{12.4167,21.2},{21,21.2}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(T_ex_Sensor.OutFlow, sinkP.flangeB) annotation (Line(
            points={{35,21.2},{44.5,21.2},{44.5,24},{51.6,24}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{
                  100,100}}), graphics={Text(
                extent={{-52,66},{-16,60}},
                lineColor={0,0,0},
                textString="Thermal port")}),
          experiment(StopTime=125, __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput);
      end Test_Flow1D_MD;

      model Test_Flow1D_reversal
        parameter Integer N = 10;
        ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim flow1Dim(
          A=2,
          Unom_l=400,
          Unom_tp=1000,
          Unom_v=400,
          N=N,
          V=0.003,
          Mdotnom=0.3,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_smooth,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          pstart=500000,
          Tstart_inlet=323.15,
          Tstart_outlet=373.15)
          annotation (Placement(transformation(extent={{-22,16},{-2,36}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
          Mdot_0=0.3,
          UseT=false,
          h_0=2E5,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          p=500000,
          T_0=293.15)
          annotation (Placement(transformation(extent={{-80,16},{-60,36}})));
        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-24,48},{-4,68}})));
        Modelica.Blocks.Sources.Constant const(k=273.15 + 140)
          annotation (Placement(transformation(extent={{-66,72},{-46,92}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(h=4e5, redeclare
            package Medium = ThermoCycle.Media.R245fa_CP)
          annotation (Placement(transformation(extent={{34,16},{54,36}})));
        Modelica.Blocks.Sources.Ramp ramp(
          offset=5E5,
          duration=5,
          startTime=5,
          height=8E5) annotation (Placement(transformation(extent={{14,56},{34,76}})));
        Modelica.Blocks.Sources.Ramp ramp1(
          duration=5,
          startTime=5,
          offset=0.3,
          height=-0.6)
                      annotation (Placement(transformation(extent={{-112,44},{-92,64}})));
      equation
        connect(sourceMdot.flangeB, flow1Dim.InFlow) annotation (Line(
            points={{-61,26},{-20.3333,26}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(source_T.thermalPort, flow1Dim.Wall_int) annotation (Line(
            points={{-14.1,53.9},{-14.1,43.95},{-12,43.95},{-12,30.1667}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(const.y, source_T.Temperature) annotation (Line(
            points={{-45,82},{-28,82},{-28,80},{-14,80},{-14,62}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, sinkP.flangeB) annotation (Line(
            points={{-3.66667,26.0833},{12,26.0833},{12,26},{35.6,26}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, sinkP.in_p0) annotation (Line(
            points={{35,66},{40,66},{40,34.8}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(ramp1.y, sourceMdot.in_Mdot) annotation (Line(
            points={{-91,54},{-76,54},{-76,32}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}),     graphics),
          experiment(StopTime=50, __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput);
      end Test_Flow1D_reversal;

      model Test_Flow1D_smoothed
        parameter Integer N = 11;
        replaceable package Medium = ThermoCycle.Media.R134a_CP
        constrainedby Modelica.Media.Interfaces.PartialMedium;
        ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim flow1Dim(
          N=N,
          redeclare package Medium = Medium,
          Mdotnom=0.2,
          A=0.25,
          V=0.002,
          Unom_tp=3000,
          filter_dMdt=true,
          max_der=true,
          Unom_l=1500,
          Unom_v=1500,
          pstart=500000,
          Tstart_inlet=218.15,
          Tstart_outlet=328.15,
          redeclare model Flow1DimHeatTransferModel =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.MassFlowDependence)
          annotation (Placement(transformation(extent={{-20,-20},{20,20}})));

        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          redeclare package Medium = Medium,
          UseT=false,
          h_0=1.3e5,
          Mdot_0=0.15,
          p=500000,
          T_0=356.26)
          annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(
          redeclare package Medium = Medium,
          h=254381,
          p0=500000)
          annotation (Placement(transformation(extent={{60,-10},{80,10}})));
        ThermoCycle.Components.FluidFlow.Sources.SourceMdot2 sourceMdot2_1
          annotation (Placement(transformation(extent={{-98,12},{-78,32}})));
        ThermoCycle.Interfaces.HeatTransfer.HeatPortConverter heatPortConverter
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={0,76})));
        Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow fixedHeatFlow(
          alpha=50,
          Q_flow=50000,
          T_ref=403.15)
          annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
        ThermoCycle.Components.HeatFlow.Walls.MetalWall metalWall(
          N=N,
          Aext=1.5*flow1Dim.A,
          Aint=flow1Dim.A,
          M_wall=1,
          c_wall=500,
          Tstart_wall_1=fixedHeatFlow.T_ref,
          Tstart_wall_end=fixedHeatFlow.T_ref,
          steadystate_T_wall=false)
          annotation (Placement(transformation(extent={{-20,48},{20,8}})));
        ThermoCycle.Interfaces.HeatTransfer.ThermalPortMultiplier thermalPortMultiplier(N=N)
          annotation (Placement(transformation(extent={{-10,60},{10,40}})));
      equation
        connect(sourceMdot1.flangeB, flow1Dim.InFlow) annotation (Line(
            points={{-61,0},{-16.6667,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, sinkP.flangeB) annotation (Line(
            points={{16.6667,0.166667},{18,0.166667},{18,0},{61.6,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sourceMdot2_1.y, sourceMdot1.in_Mdot) annotation (Line(
            points={{-79,23},{-76,23},{-76,6}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(fixedHeatFlow.port, heatPortConverter.heatPort) annotation (Line(
            points={{-40,80},{-20,80},{-20,86},{1.83697e-015,86}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(metalWall.Wall_int, flow1Dim.Wall_int) annotation (Line(
            points={{0,22},{0,8.33333}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(heatPortConverter.thermalPortL, thermalPortMultiplier.single)
          annotation (Line(
            points={{-1.83697e-015,66},{-1.83697e-015,60.05},{0,60.05},{0,54.1}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(thermalPortMultiplier.multi, metalWall.Wall_ext) annotation (Line(
            points={{0,46.5},{0,34},{-0.4,34}},
            color={255,0,0},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                  100,100}}), graphics),
          experiment(
            StopTime=500,
            __Dymola_NumberOfIntervals=1000,
            __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput);
      end Test_Flow1D_smoothed;

      model Test_Flow1DConst
      parameter Integer N = 5;
      ThermoCycle.Components.FluidFlow.Pipes.Flow1DConst flowConst(N=N)
          annotation (Placement(transformation(extent={{20,94},{-42,44}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot
                                                     source_Cdot(
          cp=1978,
          rho=928.2,
          Mdot_0=3,
          T_0=418.15)
          annotation (Placement(transformation(extent={{50,68},{70,88}})));
        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-52,-4},{-32,16}})));
        Modelica.Blocks.Sources.Constant const(k=25 + 273.15)
          annotation (Placement(transformation(extent={{-90,4},{-70,24}})));
      equation
        connect(source_Cdot.flange, flowConst.flange_Cdot) annotation (Line(
            points={{68.2,77.9},{74,77.9},{74,69},{20,69}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(const.y,source_T. Temperature) annotation (Line(
            points={{-69,14},{-42,14},{-42,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(source_T.thermalPort, flowConst.Wall_int) annotation (Line(
            points={{-42.1,1.9},{-42.1,-10},{-11,-10},{-11,56.5}},
            color={255,0,0},
            smooth=Smooth.None));
        annotation (Diagram(graphics),
          experiment(StopTime=100),
          __Dymola_experimentSetupOutput);
      end Test_Flow1DConst;

      model Test_Flow1Dinc
      parameter Integer N = 10;
      ThermoCycle.Components.FluidFlow.Pipes.Flow1DimInc       HotFluid(
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
          A=16.18,
          V=0.03781,
          Unom=30.21797814,
          Mdotnom=0.25877,
          N=N,
          steadystate=false,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.centr_diff,
          pstart=100000,
          Tstart_inlet=418.15,
          Tstart_outlet=408.15)
          annotation (Placement(transformation(extent={{-20,94},{28,42}})));
          //Tstart_inlet=402.157968,
          //Tstart_outlet=357.76567852)
          //Tstart_inlet=353.82,
          //Tstart_outlet=316.91)
       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot
                 sourceWF1(
          h_0=470523,
          UseT=true,
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
          Mdot_0=3.1,
          p=100000,
          T_0=418.15)
          annotation (Placement(transformation(extent={{-90,62},{-70,82}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP      sinkPFluid1(redeclare
            package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66, p0=
              100000)
          annotation (Placement(transformation(extent={{74,50},{94,70}})));
        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-54,10},{-34,30}})));
        Modelica.Blocks.Sources.Constant const(k=25 + 273.15)
          annotation (Placement(transformation(extent={{-92,18},{-72,38}})));
      equation
        connect(source_T.thermalPort, HotFluid.Wall_int) annotation (Line(
            points={{-44.1,15.9},{-44.1,0},{4,0},{4,57.1667}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(const.y, source_T.Temperature) annotation (Line(
            points={{-71,28},{-44,28},{-44,24}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sourceWF1.flangeB, HotFluid.InFlow) annotation (Line(
            points={{-71,72},{-60,72},{-60,76},{-44,76},{-44,68},{-16,68}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(HotFluid.OutFlow, sinkPFluid1.flangeB) annotation (Line(
            points={{24,67.7833},{40,67.7833},{40,60},{75.6,60}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(graphics),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Test_Flow1Dinc;

      model Test_Flow1Dinc_CP
      parameter Integer N = 10;
      ThermoCycle.Components.FluidFlow.Pipes.Flow1DimInc       HotFluid(
          A=16.18,
          V=0.03781,
          Unom=30.21797814,
          Mdotnom=0.25877,
          N=N,
          steadystate=false,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.centr_diff,
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleCP.HighTemperature.Therminol_66,
          pstart=100000,
          Tstart_inlet=418.15,
          Tstart_outlet=408.15)
          annotation (Placement(transformation(extent={{-20,94},{28,42}})));

          //Tstart_inlet=402.157968,
          //Tstart_outlet=357.76567852)
          //Tstart_inlet=353.82,
          //Tstart_outlet=316.91)
       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot
                 sourceWF1(
          h_0=470523,
          Mdot_0=3,
          UseT=true,
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleCP.HighTemperature.Therminol_66,
          p=100000,
          T_0=418.15)
          annotation (Placement(transformation(extent={{-90,62},{-70,82}})));

        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP      sinkPFluid1(redeclare
            package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleCP.HighTemperature.Therminol_66,
            p0=100000)
          annotation (Placement(transformation(extent={{74,50},{94,70}})));
        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-54,10},{-34,30}})));
        Modelica.Blocks.Sources.Constant const(k=25 + 273.15)
          annotation (Placement(transformation(extent={{-92,18},{-72,38}})));
      equation
        connect(source_T.thermalPort, HotFluid.Wall_int) annotation (Line(
            points={{-44.1,15.9},{-44.1,0},{4,0},{4,57.1667}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(const.y, source_T.Temperature) annotation (Line(
            points={{-71,28},{-44,28},{-44,24}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sourceWF1.flangeB, HotFluid.InFlow) annotation (Line(
            points={{-71,72},{-60,72},{-60,76},{-44,76},{-44,68},{-16,68}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(HotFluid.OutFlow, sinkPFluid1.flangeB) annotation (Line(
            points={{24,67.7833},{40,67.7833},{40,60},{75.6,60}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                            graphics),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Test_Flow1Dinc_CP;

      model Test_HeatTransferTester
        extends Modelica.Icons.Example;

      model InputSelector
      replaceable package Medium = ThermoCycle.Media.DummyFluid constrainedby
            Modelica.Media.Interfaces.PartialTwoPhaseMedium "Medium"
          annotation(choicesAllMatching=true);

      // Settings for heat transfer
      Medium.ThermodynamicState state(phase(start=0));
      // Settings for correlation
      parameter Modelica.SIunits.MassFlowRate m_dot_nom = 1 "Nomnial Mass flow rate"
                                 annotation (Dialog(tab="Heat transfer"));
      parameter Modelica.SIunits.CoefficientOfHeatTransfer U_nom_l = 2000
            "Nominal heat transfer coefficient liquid side"
                                                        annotation (Dialog(tab="Heat transfer"));
      parameter Modelica.SIunits.CoefficientOfHeatTransfer U_nom_tp = 3000
            "Nominal heat transfer coefficient two phase side"
                                                           annotation (Dialog(tab="Heat transfer"));
      parameter Modelica.SIunits.CoefficientOfHeatTransfer U_nom_v = 1500
            "Nominal heat transfer coefficient vapor side"
                                                       annotation (Dialog(tab="Heat transfer"));
      Medium.AbsolutePressure p;
      Medium.SpecificEnthalpy h;
      Medium.SpecificEnthalpy h_start;
      Medium.SpecificEnthalpy h_end;
      Modelica.SIunits.MassFlowRate m_dot "Inlet massflow";
      Real x "Vapor quality";
      Real y "Relative position";
      Modelica.SIunits.Time c = 10;

      Medium.ThermodynamicState bubbleState(h(start=0));
          Medium.ThermodynamicState dewState(h(start=0));

      replaceable model HeatTransfer =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.MassFlowDependence
        constrainedby
            ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.BaseClasses.PartialConvectiveCorrelation
            "Heat transfer model"
          annotation(choicesAllMatching=true);

          HeatTransfer heatTransfer(
        redeclare final package Medium = Medium,
        final n = 1,
        FluidState = {state},
        Mdotnom = m_dot_nom,
        Unom_l =  U_nom_l,
        Unom_tp = U_nom_tp,
        Unom_v =  U_nom_v,
        M_dot = m_dot,
        x = x);

          parameter Medium.AbsolutePressure p_start = 1e5 "Start pressure";
          parameter Medium.AbsolutePressure p_end = p_start "Final pressure";

          parameter Modelica.SIunits.MassFlowRate m_dot_start = 1 "Start flow rate";
          parameter Modelica.SIunits.MassFlowRate m_dot_end = m_dot_start
            "Final flow rate";

          parameter Boolean twoPhase = false "is two-phase medium?";
          parameter Medium.SpecificEnthalpy h_start_in = 0 "Start enthalpy"
            annotation(Dialog(enable = not twoPhase));

          parameter Medium.SpecificEnthalpy h_end_in = h_start_in "Final enthalpy"
            annotation(Dialog(enable = not twoPhase));

        //parameter Discretizations Discretization=ThermoCycle.Functions.Enumerations.Discretizations.centr_diff
        //  "Selection of the spatial discretization scheme"  annotation (Dialog(tab="Numerical options"));

      // Settings for heat transfer
      // replaceable package Medium=Modelica.Media.Interfaces.PartialMedium
      // input Medium.ThermodynamicState[n] FluidState
      // Settings for correlation
      // input Modelica.SIunits.MassFlowRate Mdotnom "Nomnial Mass flow rate";
      // input Modelica.SIunits.CoefficientOfHeatTransfer Unom_l
      //     "Nominal heat transfer coefficient liquid side";
      // input Modelica.SIunits.CoefficientOfHeatTransfer Unom_tp
      //     "nominal heat transfer coefficient two phase side";
      // input Modelica.SIunits.CoefficientOfHeatTransfer Unom_v
      //     "nominal heat transfer coefficient vapor side";
      // input Modelica.SIunits.MassFlowRate M_dot "Inlet massflow";
      // input Modelica.SIunits.QualityFactor x "Vapor quality";

      equation
        if twoPhase then
          bubbleState = Medium.setBubbleState(Medium.setSat_p(Medium.pressure(state)));
          dewState    = Medium.setDewState(   Medium.setSat_p(Medium.pressure(state)));
          x           = (Medium.specificEnthalpy(state) - Medium.specificEnthalpy(bubbleState))/(Medium.specificEnthalpy(dewState) - Medium.specificEnthalpy(bubbleState));
          h_start     = Medium.specificEnthalpy(bubbleState) - 0.25*(Medium.specificEnthalpy(dewState) - Medium.specificEnthalpy(bubbleState));
          h_end       = Medium.specificEnthalpy(dewState)    + 0.25*(Medium.specificEnthalpy(dewState) - Medium.specificEnthalpy(bubbleState));
        else
          bubbleState = state;
          dewState    = state;
          x           = 0;
          h_start     = h_start_in;
          h_end       = h_end_in;
        end if;
        y = time/c;
        p     = (1-y) * p_start     + y * p_end;
        m_dot = (1-y) * m_dot_start + y * m_dot_end;
        h     = (1-y) * h_start     + y * h_end;
        state = Medium.setState_phX(p=p,h=h);

      end InputSelector;

        InputSelector tester(
          h_start_in=100e3,
          twoPhase=true,
          redeclare package Medium = ThermoCycle.Media.R134a_CP(substanceNames={"R134a|debug=0|calc_transport=1|enable_EXTTP=1|enable_TTSE=0"}),
          m_dot_start=3,
          m_dot_nom=3,
          redeclare model HeatTransfer =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Smoothed,
          p_start=500000)
          annotation (Placement(transformation(extent={{-42,42},{-22,62}})));

        annotation (experiment(StopTime=10));
      end Test_HeatTransferTester;

      model Test_Hx1D
      // parameter Real k( start = 1, fixed = false)= 1;
      //
      // Modelica.SIunits.Power PowerRec = Recuperator.Q_hot_;

        ThermoCycle.Components.Units.HeatExchangers.Hx1D    Recuperator(
          redeclare package Medium1 = CoolProp2Modelica.Media.SES36_CP,
          redeclare package Medium2 = CoolProp2Modelica.Media.SES36_CP,
          MdotNom_Hot=0.3335,
          MdotNom_Cold=0.3335,
          steadystate_h_cold=true,
          N=10,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.centr_diff,
          Unom_l_cold=1500,
          Unom_tp_cold=1500,
          Unom_v_cold=1500,
          Unom_l_hot=1000,
          Unom_tp_hot=1000,
          Unom_v_hot=1000,
          pstart_cold=863885,
          pstart_hot=127890,
          Tstart_inlet_cold=303.59,
          Tstart_outlet_cold=356.26,
          Tstart_inlet_hot=368.05,
          Tstart_outlet_hot=315.81)
          annotation (Placement(transformation(extent={{-34,-22},{28,40}})));

        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdot_0=0.3335,
          p=888343,
          T_0=303.59)
          annotation (Placement(transformation(extent={{-94,-28},{-74,-8}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium =
                     CoolProp2Modelica.Media.SES36_CP,
          h=290659,
          p0=863885)
          annotation (Placement(transformation(extent={{68,-48},{88,-28}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdot_0=0.3335,
          UseT=false,
          h_0=236271,
          p=127890,
          T_0=368.05)
          annotation (Placement(transformation(extent={{26,70},{46,90}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          h=363652,
          p0=127890)
          annotation (Placement(transformation(extent={{-54,72},{-36,90}})));
      // initial equation
      // PowerRec = 20.330e3;

        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp(redeclare package
            Medium = CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{-32,18},{-52,38}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp1(redeclare
            package Medium =
                     CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{30,-12},{50,8}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp2(redeclare
            package Medium =
                     CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{-62,-22},{-42,-2}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp3(redeclare
            package Medium =
                     CoolProp2Modelica.Media.SES36_CP)
          annotation (Placement(transformation(extent={{40,34},{60,54}})));
      equation

        connect(Recuperator.outlet_fl1, sensTp1.InFlow) annotation (Line(
            points={{17.6667,-1.33333},{26,-1.33333},{26,-6.8},{33,-6.8}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp1.OutFlow, sinkP.flangeB) annotation (Line(
            points={{47,-6.8},{60,-6.8},{60,-38},{69.6,-38}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp.InFlow, Recuperator.outlet_fl2) annotation (Line(
            points={{-35,23.2},{-30,20.9867},{-23.2533,20.9867}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp.OutFlow, sinkP1.flangeB) annotation (Line(
            points={{-49,23.2},{-66,23.2},{-66,81},{-52.56,81}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sourceMdot.flangeB, sensTp2.InFlow) annotation (Line(
            points={{-75,-18},{-66,-18},{-66,-16.8},{-59,-16.8}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp2.OutFlow, Recuperator.inlet_fl1) annotation (Line(
            points={{-45,-16.8},{-32,-16.8},{-32,-1.33333},{-23.6667,-1.33333}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sourceMdot1.flangeB, sensTp3.InFlow) annotation (Line(
            points={{45,80},{54,80},{54,58},{32,58},{32,39.2},{43,39.2}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp3.OutFlow, Recuperator.inlet_fl2) annotation (Line(
            points={{57,39.2},{68,39.2},{68,21.4},{17.2533,21.4}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}}),
                            graphics), Icon(coordinateSystem(extent={{-100,-100},
                  {100,100}})),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Test_Hx1D;

      model Test_Hx1DConst

      ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
            package Medium =
                     ThermoCycle.Media.R245fa_CP,  p0=2357000)
          annotation (Placement(transformation(extent={{82,-10},{102,10}})));
      ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          Mdot_0=0.2588,
          h_0=281455,
          UseT=true,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          p=2357000,
          T_0=353.15)
          annotation (Placement(transformation(extent={{-92,-10},{-72,10}})));
       ThermoCycle.Components.Units.HeatExchangers.Hx1DConst hx1DConst(
          redeclare package Medium1 = ThermoCycle.Media.R245fa_CP,
          steadystate_T_sf=true,
          steadystate_h_wf=true,
          steadystate_T_wall=true,
          N=10,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind,
          SecondaryFluid(Discretization=ThermoCycle.Functions.Enumerations.Discretizations.centr_diff),
          counterCurrent=false)
          annotation (Placement(transformation(extent={{-30,-2},{2,36}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot   source_Cdot(
          cp=1978,
          rho=928.2,
          Mdot_0=3,
          T_0=418.15)
          annotation (Placement(transformation(extent={{-20,46},{0,66}})));
      equation
        connect(sourceWF.flangeB, hx1DConst.inletWf)
                                                   annotation (Line(
            points={{-73,0},{-66,0},{-66,-2},{-56,-2},{-56,7.5},{-30,7.5}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(hx1DConst.outletWf, sinkPFluid.flangeB)
                                                      annotation (Line(
            points={{2,7.5},{22,7.5},{22,6},{36,6},{36,-2},{83.6,-2},{83.6,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(source_Cdot.flange, hx1DConst.inletSf)
                                                     annotation (Line(
            points={{-1.8,55.9},{36,55.9},{36,26.5},{2,26.5}},
            color={255,0,0},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                preserveAspectRatio=true),
                            graphics), Icon(coordinateSystem(extent={{-100,-100},
                  {100,100}})),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Test_Hx1DConst;

      model Test_Hx1DInc
        ThermoCycle.Components.Units.HeatExchangers.Hx1DInc hx1DInc(
          Mdotnom_sf=3.148,
          redeclare package Medium1 = CoolProp2Modelica.Media.SES36_CP,
          Mdotnom_wf=0.3335,
          steadystate_h_wf=true,
          N=10,
          Unom_sf=900,
          Mdotconst_wf=false,
          max_der_wf=false,
          Unom_l=3000,
          Unom_tp=3600,
          Unom_v=3000,
          redeclare package Medium2 =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
          pstart_sf=100000,
          pstart_wf=888343,
          Tstart_inlet_wf=356.26,
          Tstart_outlet_wf=397.75,
          Tstart_inlet_sf=398.15,
          Tstart_outlet_sf=389.45)
          annotation (Placement(transformation(extent={{-38,-24},{24,38}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdot_0=0.3335,
          UseT=false,
          h_0=84867,
          p=888343,
          T_0=356.26)
          annotation (Placement(transformation(extent={{-90,-28},{-70,-8}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium =
                     CoolProp2Modelica.Media.SES36_CP,
          h=254381,
          p0=888343)
          annotation (Placement(transformation(extent={{68,-48},{88,-28}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          Mdot_0=3.148,
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
          p=100000,
          T_0=398.15)
          annotation (Placement(transformation(extent={{26,70},{46,90}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(redeclare
            package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66, p0=
              100000)
          annotation (Placement(transformation(extent={{-54,72},{-36,90}})));
      equation
        connect(hx1DInc.outlet_fl1, sinkP.flangeB) annotation (Line(
            points={{16.8462,-4.92308},{40,-4.92308},{40,-38},{69.6,-38}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sourceMdot1.flangeB, hx1DInc.inlet_fl2) annotation (Line(
            points={{45,80},{60,80},{60,21.3077},{16.3692,21.3077}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinkP1.flangeB, hx1DInc.outlet_fl2) annotation (Line(
            points={{-52.56,81},{-80,81},{-80,20.8308},{-30.3692,20.8308}},
            color={0,0,255},
            smooth=Smooth.None));

        connect(sourceMdot.flangeB, hx1DInc.inlet_fl1) annotation (Line(
            points={{-71,-18},{-44,-18},{-44,-4.92308},{-30.8462,-4.92308}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(graphics),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Test_Hx1DInc;

      model Test_MBeva
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
          UseT=false,
          h_0=852450,
          Mdot_0=0.05,
          redeclare package Medium = ThermoCycle.Media.Water,
          p=6000000)
          annotation (Placement(transformation(extent={{-84,-54},{-64,-34}})));
        Modelica.Blocks.Sources.Constant Hin(k=852540)
          annotation (Placement(transformation(extent={{-100,-16},{-80,4}},
                rotation=0)));
        Modelica.Blocks.Sources.Constant Pout(k=1e5)
          annotation (Placement(transformation(extent={{110,0},{90,20}}, rotation=
                 0)));
       ThermoCycle.Components.Units.HeatExchangers.MBeva MB_eva(
          A=0.0314159,
          L=1,
          M_tot=9.35E+01,
          c_wall=385,
          rho_wall=8.93e3,
          TwSB(start=510.97 + 50, displayUnit="K"),
          TwTP(start=548.79 + 21, displayUnit="K"),
          TwSH(start=585.97 + 35, displayUnit="K"),
          Tsf_SU_start=360 + 273.15,
          U_SB=3000,
          U_TP=10000,
          U_SH=3000,
          L_SB(start=0.2),
          L_TP(start=0.4),
          h_EX(start=3043000),
          Void=0.665,
          dVoid_dp=0,
          dVoid_dh=0,
          Usf=20000,
          ETA=1,
          redeclare package Medium = ThermoCycle.Media.Water,
          p(start=6000000),
          dTsf_start=343.15)
          annotation (Placement(transformation(extent={{-26,-38},{22,10}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceCdot sourceCdot(
          rho=1000,
          Mdot_0=2,
          cp=2046,
          T_0=653.15) annotation (Placement(transformation(extent={{2,30},{22,50}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium =
              ThermoCycle.Media.Water)
          annotation (Placement(transformation(extent={{76,-34},{96,-14}})));
        ThermoCycle.Components.Units.PdropAndValves.Valve valve(
          UseNom=true,
          redeclare package Medium = ThermoCycle.Media.Water,
          Mdot_nom=0.05,
          p_nom=6000000,
          T_nom=643.15,
          DELTAp_nom=5000000)
          annotation (Placement(transformation(extent={{46,-40},{66,-20}})));
      equation
        connect(Hin.y, sourceMdot.in_h) annotation (Line(
            points={{-79,-6},{-68,-6},{-68,-38}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(Pout.y, sinkP.in_p0) annotation (Line(
            points={{89,10},{82,10},{82,-15.2}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sourceCdot.flange, MB_eva.InFlow_sf) annotation (Line(
            points={{20.2,39.9},{34,39.9},{34,0.4},{22,0.4}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(sourceMdot.flangeB, MB_eva.InFlow) annotation (Line(
            points={{-65,-44},{-58,-44},{-58,-40},{-50,-40},{-50,-29.36},{-25.52,
                -29.36}},
            color={0,0,255},
            smooth=Smooth.None));

        connect(MB_eva.OutFlow, valve.InFlow) annotation (Line(
            points={{22,-29.36},{40,-29.36},{40,-30},{47,-30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(valve.OutFlow, sinkP.flangeB) annotation (Line(
            points={{65,-30},{72,-30},{72,-24},{77.6,-24}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,60}},
                preserveAspectRatio=true),
                            graphics), Icon(coordinateSystem(extent={{-100,-100},{100,
                  60}})),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Test_MBeva;

      model Test_Nozzle

        ThermoCycle.Components.Units.PdropAndValves.Nozzle
          leakageNozzle(
          redeclare package Medium = Media.R718_CP,
          Afull=5e-5,
          gamma_start=1.3,
          Use_gamma=false,
          P_su_start=1500000,
          T_su_start=623.15,
          P_ex_start=1400000)
          annotation (Placement(transformation(extent={{-10,-14},{10,6}})));
        Modelica.Blocks.Sources.Sine sine(
          freqHz=0.25,
          phase=0,
          amplitude=4.1e5,
          offset=12e5)
          annotation (Placement(transformation(extent={{-92,36},{-72,56}})));
        Modelica.Fluid.Sources.Boundary_ph boundary1(
          nPorts=1,
          use_p_in=true,
          redeclare package Medium = Media.R718_CP)
          annotation (Placement(transformation(extent={{80,-14},{60,6}})));
        Modelica.Fluid.Sources.Boundary_pT boundary(
          nPorts=1,
          redeclare package Medium = Media.R718_CP,
          p=1500000,
          T=623.15) annotation (Placement(transformation(extent={{-80,-14},{-60,6}})));
      equation
        connect(sine.y,boundary1. p_in) annotation (Line(
            points={{-71,46},{88,46},{88,4},{82,4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(boundary.ports[1],leakageNozzle. su) annotation (Line(
            points={{-60,-4},{-36,-4},{-36,-3.8},{-10.2,-3.8}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(leakageNozzle.ex,boundary1. ports[1]) annotation (Line(
            points={{10,-3.8},{36,-3.8},{36,-4},{60,-4}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=10),
          __Dymola_experimentSetupOutput);
      end Test_Nozzle;

      model Test_OpenTank

       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          h_0=2.4E5,
          redeclare package Medium = ThermoCycle.Media.Water,
          Mdot_0=3,
          UseT=true,
          p=200000,
          T_0=313.15)
                    annotation (Placement(transformation(extent={{-76,46},{-56,66}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SinkVdot sinkVdot(
          redeclare package Medium = ThermoCycle.Media.Water,
          h_out=167691,
          Vdot_0=0.001,
          pstart=200000)
          annotation (Placement(transformation(extent={{46,26},{66,46}})));
        ThermoCycle.Components.Units.Tanks.OpenTank
                                        openTank(
          redeclare package Medium = ThermoCycle.Media.Water,
          H_D=2.5,
          V_tank=4,
          L_lstart=0.3,
          Mdotnom=2,
          p_ext=300000,
          Tstart=373.15)
          annotation (Placement(transformation(extent={{-16,38},{4,58}})));
        ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTp(redeclare package
            Medium = ThermoCycle.Media.Water)
          annotation (Placement(transformation(extent={{12,28},{32,48}})));
      equation
        connect(sourceWF.flangeB, openTank.InFlow) annotation (Line(
            points={{-57,56},{-50,56},{-50,42},{-15.8,42},{-15.8,39.6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sensTp.OutFlow, sinkVdot.flangeB) annotation (Line(
            points={{29,33.2},{37.5,33.2},{37.5,36},{46.2,36}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(openTank.OutFlow, sensTp.InFlow) annotation (Line(
            points={{3.8,39.6},{10,39.6},{10,33.2},{15,33.2}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,0},{100,
                  100}}), graphics),
          experiment(StopTime=100),
          __Dymola_experimentSetupOutput,
          Icon(coordinateSystem(extent={{-100,0},{100,100}})));
      end Test_OpenTank;

      model Test_Pump

        ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump
                                                pump(
          PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.UD,
          X_pp0=1,
          eta_em=1,
          eta_is=1,
          epsilon_v=1,
          f_pp0=50,
          V_dot_max=0.0039,
          M_dot_start=3,
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
          PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.FF)
          annotation (Placement(transformation(extent={{-14,-16},{12,10}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceP sourceP(
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,

          p0=115794,
          T_0=389.15)
          annotation (Placement(transformation(extent={{-100,-16},{-80,4}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.Therminol66,
            p0=150000)
          annotation (Placement(transformation(extent={{54,2},{74,22}})));
        Modelica.Blocks.Sources.Constant const(k=0.5)
          annotation (Placement(transformation(extent={{-62,42},{-42,62}})));
      equation
        connect(sourceP.flange, pump.InFlow) annotation (Line(
            points={{-80.6,-6},{-46,-6},{-46,-2.35},{-10.36,-2.35}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pump.OutFlow, sinkP.flangeB) annotation (Line(
            points={{6.28,6.62},{30.14,6.62},{30.14,12},{55.6,12}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(const.y, pump.flow_in) annotation (Line(
            points={{-41,52},{-18,52},{-18,54},{-5.16,54},{-5.16,7.4}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(graphics));
      end Test_Pump;

      model Test_SolaFieldSchott

       ThermoCycle.Components.Units.Solar.SolarField_SchottSopo         solarCollectorIncSchott(
          Mdotnom=0.5,
          redeclare model FluidHeatTransferModel =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Ideal,
          redeclare
            ThermoCycle.Components.HeatFlow.Walls.SolarAbsorber.Geometry.Schott_SopoNova.Schott_2008_PTR70_Vacuum
            CollectorGeometry,
          redeclare package Medium1 = ThermoCycle.Media.Water,
          Ns=2,
          Tstart_inlet=298.15,
          Tstart_outlet=373.15,
          pstart=1000000)
          annotation (Placement(transformation(extent={{-34,-28},{8,42}})));

       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(Mdot_0=0.5,
          redeclare package Medium = ThermoCycle.Media.Water,
          p=1000000)
          annotation (Placement(transformation(extent={{-66,-70},{-46,-50}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium =
                     ThermoCycle.Media.Water, p0=1000000)
          annotation (Placement(transformation(extent={{22,56},{42,76}})));
        Modelica.Blocks.Sources.Constant const(k=25 + 273.15)
          annotation (Placement(transformation(extent={{-94,-12},{-74,8}})));
        Modelica.Blocks.Sources.Constant const1(k=0)
          annotation (Placement(transformation(extent={{-94,16},{-74,36}})));
        Modelica.Blocks.Sources.Constant const3(k=0)
          annotation (Placement(transformation(extent={{-92,48},{-72,68}})));
        Modelica.Blocks.Sources.Step step(
          offset=850,
          startTime=100,
          height=0)
          annotation (Placement(transformation(extent={{-92,-40},{-72,-20}})));
      equation
        connect(sourceMdot.flangeB, solarCollectorIncSchott.InFlow) annotation (
            Line(
            points={{-47,-60},{-6,-60},{-6,-28.6364}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinkP.flangeB, solarCollectorIncSchott.OutFlow) annotation (Line(
            points={{23.6,66},{-6,66},{-6,41.3636}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(const3.y, solarCollectorIncSchott.v_wind) annotation (Line(
            points={{-71,58},{-56,58},{-56,34},{-30.7333,34},{-30.7333,35}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(const1.y, solarCollectorIncSchott.Theta) annotation (Line(
            points={{-73,26},{-48,26},{-48,21.3182},{-30.5,21.3182}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(const.y, solarCollectorIncSchott.Tamb) annotation (Line(
            points={{-73,-2},{-54,-2},{-54,6.04545},{-30.9667,6.04545}},
            color={0,0,127},
            smooth=Smooth.None));

        connect(step.y, solarCollectorIncSchott.DNI) annotation (Line(
            points={{-71,-30},{-56,-30},{-56,-12.4091},{-30.5,-12.4091}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(extent={{-100,-80},{60,100}}),
                            graphics),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput,
          Icon(coordinateSystem(extent={{-100,-80},{60,100}})));
      end Test_SolaFieldSchott;

      model Test_SolaFieldSoltigua

      parameter Real VV;
      ThermoCycle.Components.Units.Solar.SolarField_Soltigua_Inc        solarCollectorIncSchott(
          Mdotnom=0.5,
          Ns=2,
          redeclare
            ThermoCycle.Components.HeatFlow.Walls.SolarAbsorber.Geometry.Soltigua.PTMx_18
            CollectorGeometry,
          redeclare model FluidHeatTransferModel =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
          redeclare package Medium1 =
              ThermoCycle.Media.Incompressible.IncompressibleTables.TherminolSP,
          Tstart_inlet=298.15,
          Tstart_outlet=373.15,
          pstart=1000000)
          annotation (Placement(transformation(extent={{-34,-28},{8,42}})));

       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(Mdot_0=0.5,
          redeclare package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.TherminolSP,
          p=200000)
          annotation (Placement(transformation(extent={{-66,-70},{-46,-50}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(redeclare
            package Medium =
              ThermoCycle.Media.Incompressible.IncompressibleTables.TherminolSP, p0=
              200000)
          annotation (Placement(transformation(extent={{22,56},{42,76}})));
        Modelica.Blocks.Sources.Constant const(k=25 + 273.15)
          annotation (Placement(transformation(extent={{-118,-22},{-98,-2}})));
        Modelica.Blocks.Sources.Constant const1(k=0)
          annotation (Placement(transformation(extent={{-118,12},{-98,32}})));
        Modelica.Blocks.Sources.Constant const3(k=0)
          annotation (Placement(transformation(extent={{-98,44},{-78,64}})));
        Modelica.Blocks.Sources.Step step(
          startTime=100,
          height=0,
          offset=VV)
          annotation (Placement(transformation(extent={{-110,-54},{-90,-34}})));
        Modelica.Blocks.Sources.BooleanConstant booleanConstant
          annotation (Placement(transformation(extent={{-162,-74},{-142,-54}})));
      equation
        connect(sourceMdot.flangeB, solarCollectorIncSchott.InFlow) annotation (
            Line(
            points={{-47,-60},{-8.8,-60},{-8.8,-28.6364}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinkP.flangeB, solarCollectorIncSchott.OutFlow) annotation (Line(
            points={{23.6,66},{-6,66},{-6,41.3636}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(const3.y, solarCollectorIncSchott.v_wind) annotation (Line(
            points={{-77,54},{-56,54},{-56,34},{-31.2,34},{-31.2,36.2727}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(const1.y, solarCollectorIncSchott.Theta) annotation (Line(
            points={{-97,22},{-48,22},{-48,22.9091},{-31.6667,22.9091}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(const.y, solarCollectorIncSchott.Tamb) annotation (Line(
            points={{-97,-12},{-54,-12},{-54,8.90909},{-31.6667,8.90909}},
            color={0,0,127},
            smooth=Smooth.None));

        connect(step.y, solarCollectorIncSchott.DNI) annotation (Line(
            points={{-89,-44},{-56,-44},{-56,-4.13636},{-31.9,-4.13636}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(booleanConstant.y, solarCollectorIncSchott.Defocusing) annotation (
            Line(
            points={{-141,-64},{-110,-64},{-110,-68},{-68,-68},{-68,-17.8182},{
                -31.6667,-17.8182}},
            color={255,0,255},
            smooth=Smooth.None));
        annotation (Diagram(graphics),
          experiment(StopTime=1000),
          __Dymola_experimentSetupOutput);
      end Test_SolaFieldSoltigua;

      model Test_Tank_pL

      ThermoCycle.Components.Units.Tanks.Tank_pL tank_pL(
          Vtot=0.01,
          impose_L=true,
          impose_pressure=false,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          pstart=228288)
          annotation (Placement(transformation(extent={{-52,10},{-32,30}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceWF(
          UseT=false,
          h_0=2.4E5,
          Mdot_0=0.35,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          p=228288) annotation (Placement(transformation(extent={{-76,46},{-56,66}})));
       ThermoCycle.Components.FluidFlow.Reservoirs.SinkVdot sinkVdot(Vdot_0=2e-4,
          redeclare package Medium = ThermoCycle.Media.R245fa_CP,
          pstart=200000)
          annotation (Placement(transformation(extent={{-32,-40},{-12,-20}})));
      equation
        connect(sourceWF.flangeB, tank_pL.InFlow)
                                               annotation (Line(
            points={{-57,56},{-42,56},{-42,28.4}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(tank_pL.OutFlow, sinkVdot.flangeB)
                                                annotation (Line(
            points={{-42,11.2},{-42,-30},{-31.8,-30}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{20,
                  100}}), graphics), Icon(coordinateSystem(extent={{-100,-100},{20,
                  100}})));
      end Test_Tank_pL;

      model Test_Valve

        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid(redeclare
            package Medium = ThermoCycle.Media.R245fa_CP,  p0=400000)
          annotation (Placement(transformation(extent={{56,-20},{76,0}})));
        Modelica.Blocks.Sources.Ramp ramp(
          duration=2,
          startTime=1,
          height=4E5,
          offset=4E5)
          annotation (Placement(transformation(extent={{-72,26},{-52,46}})));
        Real y;
        ThermoCycle.Components.Units.PdropAndValves.Valve valve(
          UseNom=true,
          Mdot_nom=0.5,
          p_nom=400000,
          T_nom=298.15,
          DELTAp_nom=40000)
          annotation (Placement(transformation(extent={{-6,-24},{14,-4}})));
        Modelica.Blocks.Sources.Ramp ramp1(
          duration=2,
          height=-1,
          offset=1,
          startTime=10)
          annotation (Placement(transformation(extent={{-6,22},{14,42}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkPFluid1(
                                                                     redeclare
            package Medium = ThermoCycle.Media.R245fa_CP,  p0=400000)
          annotation (Placement(transformation(extent={{-34,-22},{-54,-2}})));
      equation
        y = ThermoCycle.Functions.weightingfactor(
              t_init=2,
              length=5,
              t=time)
        annotation (Diagram(graphics), uses(Modelica(version="3.2")));
        connect(valve.OutFlow, sinkPFluid.flangeB) annotation (Line(
            points={{13,-14},{38,-14},{38,-10},{57.6,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp1.y, valve.cmd) annotation (Line(
            points={{15,32},{36,32},{36,10},{4,10},{4,-6}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sinkPFluid1.flangeB, valve.InFlow) annotation (Line(
            points={{-35.6,-12},{-20,-12},{-20,-14},{-5,-14}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, sinkPFluid1.in_p0) annotation (Line(
            points={{-51,36},{-40,36},{-40,-3.2}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-80,
                  -100},{100,80}}),
                            graphics),
          experiment(StopTime=50),
          __Dymola_experimentSetupOutput,
          Icon(coordinateSystem(extent={{-80,-100},{100,80}})));
      end Test_Valve;
    annotation (Documentation(info="<HTML>
<p><big><dl><dt><b>Main Authors:</b> <br/></dt>
<dd>Sylvain Quoilin; &LT;<a href=\"squoilin@ulg.ac.be\">squoilin@ulg.ac.be</a>&GT;</dd>
<dd>Adriano Desideri &LT;<a href=\"adesideri@ulg.ac.be\">adesideri@ulg.ac.be</a>&GT;<br/></dd>
<dd>University of Liege</dd>
<dd>Laboratory of thermodynamics</dd>
<dd>Campus du Sart-Tilman Bât B49 (P33)</dd>
<dd>B-4000 Liège - BELGIUM -<br/></dd>
<dt><b>Copyright:</b> </dt>
<dd>Copyright &copy; 2013-2014, Sylvain Quoilin and Adriano Desideri.<br/></dd>
<dd><i>The IndustrialControlSystems package is <b>free</b> software; it can be redistributed and/or modified under the terms of the <b>Modelica license</b>.</i><br/></dd>
</dl></html>"));
    end TestComponents;

    package TestFluid
      extends Modelica.Icons.Package;

      model TestCP
        replaceable package Medium2 = ThermoCycle.Media.Propane_CP;
        CoolProp2Modelica.Test.TestMedium.GenericModels.CompleteBaseProperties
          mediumRP(                 redeclare package Medium = Medium2)
          "Varying pressure, constant enthalpy";
      equation
        mediumRP.baseProperties.p = 2.5e6;
        mediumRP.baseProperties.h = 0+time*200000;
        annotation (experiment(Algorithm="Dassl"),
            __Dymola_experimentSetupOutput);
      end TestCP;

      model TestFP
        replaceable package Medium2 = ThermoCycle.Media.Propane_FPRP;
        CoolProp2Modelica.Test.TestMedium.GenericModels.CompleteBaseProperties
          mediumRP(                 redeclare package Medium = Medium2)
          "Varying pressure, constant enthalpy";
      equation
        mediumRP.baseProperties.p = 2.5e6;
        mediumRP.baseProperties.h = 0+time*200000;
        annotation (experiment(Algorithm="Euler"),
            __Dymola_experimentSetupOutput);
      end TestFP;

      model TestTIL
      parameter String fluid = "Refprop.propane";
        TILMedia.Refrigerant refrigerant( refrigerantName = fluid)
          annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
      equation
        refrigerant.p = 2.5e6;
        refrigerant.h = 0 + time*200000;
        annotation (experiment(Algorithm="Euler"),
            __Dymola_experimentSetupOutput);
      end TestTIL;
    annotation (Documentation(info="<HTML>
<p><big><dl><dt><b>Main Authors:</b> <br/></dt>
<dd>Sylvain Quoilin; &LT;<a href=\"squoilin@ulg.ac.be\">squoilin@ulg.ac.be</a>&GT;</dd>
<dd>Adriano Desideri &LT;<a href=\"adesideri@ulg.ac.be\">adesideri@ulg.ac.be</a>&GT;<br/></dd>
<dd>University of Liege</dd>
<dd>Laboratory of thermodynamics</dd>
<dd>Campus du Sart-Tilman Bât B49 (P33)</dd>
<dd>B-4000 Liège - BELGIUM -<br/></dd>
<dt><b>Copyright:</b> </dt>
<dd>Copyright &copy; 2013-2014, Sylvain Quoilin and Adriano Desideri.<br/></dd>
<dd><i>The IndustrialControlSystems package is <b>free</b> software; it can be redistributed and/or modified under the terms of the <b>Modelica license</b>.</i><br/></dd>
</dl></html>"));
    end TestFluid;

    package TestFunctions "Function test files"
      extends Modelica.Icons.Package;

      model test_der_regsquare
        Real x;
        parameter Real x_small=0.01 "approximation of function for |x| <= x_small";
        parameter Real k1=1 "y = (if x>=0 then k1 else k2)*x*|x|";
        parameter Real k2=1 "y = (if x>=0 then k1 else k2)*x*|x|";
        parameter Boolean use_yd0 = false "= true, if yd0 shall be used";
        parameter Real yd0=1 "Desired derivative at x=0: dy/dx = yd0";
        Real y "ordinate value";
        Real y_bis;
      equation
      x = -1+time;
      y =Modelica.Fluid.Utilities.regSquare2(x,x_small,k1,k2,use_yd0,yd0);
      //y_der = ThermoCycle.functions.weightingfactor_der(t_init,length,time,0,0,t_der);
      der(y_bis) = der(y);
      initial equation
        y_bis = y;
        annotation (
          experiment(StopTime=3),
          __Dymola_experimentSetupOutput);
      end test_der_regsquare;

      model test_derweight
      Real t_init;
      Real length;
      Real t_der;
      Real y;
      Real y_bis;
      Real y_der;
      equation
      t_der = 1;
      t_init = 1;
      length = 1;
        y = ThermoCycle.Functions.weightingfactor(
              t_init,
              length,
              time);
        y_der = ThermoCycle.Functions.weightingfactor_der(
              t_init,
              length,
              time,
              0,
              0,
              t_der);
      der(y_bis) = der(y);
      initial equation
        y_bis = y;
        annotation (
          experiment(StopTime=3),
          __Dymola_experimentSetupOutput);
      end test_derweight;

      package TestDerivative
        extends Modelica.Icons.Package;

        function nonderivable
          input Real x;
          output Real y;
        algorithm
         // the if statement impeeds modelica to find the derivatives by itself
             if x < 0.5 then
               y:=x;
             else
               y:=x;
             end if;
        end nonderivable;

        record R
          Real field1;
          Real field2;
        end R;

        function recordFunction
          input R machin;
          input Real x;
          input Real y;
          output Real z;
        algorithm
         // the if statement impeeds modelica to find the derivatives by itself and forces the use of the derivative function
             if x < 0.5 then
               z := machin.field1*x^2 + machin.field2*y^2 + x*y;
             else
               z := machin.field1*x^2 + machin.field2*y^2 + x*y;
             end if;
        //     z := machin.field1*x^2 + machin.field2*y^2 + x*y;
          annotation (derivative=recordFunction_d);
        end recordFunction;

        function recordFunction_d
          input R machin;          // The record is considered as a variable, it derivative must be an input of the function, although is is not used
          input Real x;
          input Real y;
          //input R dummymachin;    // Dummy derivative of the record
          input Real der_x;
          input Real der_y;
          output Real der_z;
        algorithm
          der_z := machin.field1*2*x*der_x + machin.field2*2*y*der_y + x*der_y + y*der_x;
        end recordFunction_d;

        model test
        R machin;
        Real x;
        Real y;
        Real z;
        Real z_bis;
        equation
        machin.field1 = nonderivable(3);
        machin.field2 = 5;
        x = time^2;
        y = 3;
        z = recordFunction(machin,x,y);
        der(z_bis) = der(z);
        initial equation
          z_bis = z;
        end test;
        annotation ();
      end TestDerivative;

      model transition_tester
        parameter Integer n = 5;
        Real fValue "value before transition";
        Real gValue "value after transition";
        Real start "start of transition interval";
        Real stop "end of transition interval";
        Real position "current position";
        Real derF;
        Real derG "Derivatives of input functions";
        Real der0a;
        Real der0b;
        Real der0c;
        Real der0d;
        Real[n] derR08_0;
        Real[n] derR08_1;
        Real[n] derR08_2;
        Real[n] derGLF;
        Real R08_0th;
        Real R08_1st;
        Real R08_2nd;
        Real GLF_v;
        Real[n] derR08_0th;
        Real[n] derR08_1st;
        Real[n] derR08_2nd;
        Real[n] derGLF_v;
      equation
        fValue = time;
        gValue = time^3;
        start = 0.3;
        stop  = 0.6;
        position = time;
        derF = der(fValue);
        derG = der(gValue);
        der0a = ThermoCycle.Functions.transition_factor(
                                                   start=start,stop=stop,position=position,order=0);
        der0b = ThermoCycle.Functions.transition_factor(
                                                   start=start,stop=stop,position=position,order=1);
        der0c = ThermoCycle.Functions.transition_factor(
                                                   start=start,stop=stop,position=position,order=2);
        der0d = ThermoCycle.Functions.transition_factor(
                                                   start=start,stop=stop,position=position,order=3);
        R08_0th = der0a*fValue + (1 - der0a)*gValue;
        R08_1st = der0b*fValue + (1 - der0b)*gValue;
        R08_2nd = der0c*fValue + (1 - der0c)*gValue;
        GLF_v   = der0d*fValue + (1 - der0d)*gValue;
        derR08_0[1] = der(der0a);
        derR08_1[1] = der(der0b);
        derR08_2[1] = der(der0c);
        derGLF[1]   = der(der0d);
        derR08_0th[1] = der(R08_0th);
        derR08_1st[1] = der(R08_1st);
        derR08_2nd[1] = der(R08_2nd);
        derGLF_v[1]   = der(GLF_v);
        for i in 2:n loop
          // factors
          derR08_0[i] = der(derR08_0[i-1]);
          derR08_1[i] = der(derR08_1[i-1]);
          derR08_2[i] = der(derR08_2[i-1]);
          derGLF[i]   = der(derGLF[i-1]);
          // and values
          derR08_0th[i] = der(derR08_0th[i-1]);
          derR08_1st[i] = der(derR08_1st[i-1]);
          derR08_2nd[i] = der(derR08_2nd[i-1]);
          derGLF_v[i]   = der(derGLF_v[i-1]);
        end for;
      end transition_tester;
    annotation (Documentation(info="<HTML>
<p><big><dl><dt><b>Main Authors:</b> <br/></dt>
<dd>Sylvain Quoilin; &LT;<a href=\"squoilin@ulg.ac.be\">squoilin@ulg.ac.be</a>&GT;</dd>
<dd>Adriano Desideri &LT;<a href=\"adesideri@ulg.ac.be\">adesideri@ulg.ac.be</a>&GT;<br/></dd>
<dd>University of Liege</dd>
<dd>Laboratory of thermodynamics</dd>
<dd>Campus du Sart-Tilman Bât B49 (P33)</dd>
<dd>B-4000 Liège - BELGIUM -<br/></dd>
<dt><b>Copyright:</b> </dt>
<dd>Copyright &copy; 2013-2014, Sylvain Quoilin and Adriano Desideri.<br/></dd>
<dd><i>The IndustrialControlSystems package is <b>free</b> software; it can be redistributed and/or modified under the terms of the <b>Modelica license</b>.</i><br/></dd>
</dl></html>"));
    end TestFunctions;

    package TestNumericalMethods
                                 extends Modelica.Icons.Package;
      model Cell1D_Trunc

        ThermoCycle.Components.FluidFlow.Pipes.Cell1Dim flow1Dim(
          Ai=0.2,
          Unom_l=400,
          Unom_tp=1000,
          Unom_v=400,
          max_drhodt=50,
          Vi=0.005,
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdotnom=0.3335,
          hstart=84867,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind,

          pstart=866735,
          Mdotconst=true)
          annotation (Placement(transformation(extent={{-24,8},{-4,28}})));
        ThermoCycle.Components.HeatFlow.Sources.Source_T_cell source_T
          annotation (Placement(transformation(extent={{-24,44},{-4,64}})));
        Modelica.Blocks.Sources.Constant const(k=273.15 + 140)
          annotation (Placement(transformation(extent={{-66,72},{-46,92}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdot_0=0.3335,
          UseT=false,
          h_0=84867,
          p=888343,
          T_0=356.26)
          annotation (Placement(transformation(extent={{-90,12},{-70,32}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          h=254381,
          p0=866735)
          annotation (Placement(transformation(extent={{40,4},{60,24}})));
        Modelica.Blocks.Sources.Sine sine(
          startTime=10,
          offset=0.3335,
          amplitude=0.5,
          phase=0,
          freqHz=0.1)
          annotation (Placement(transformation(extent={{-114,44},{-100,58}})));
      equation
        connect(const.y, source_T.Temperature) annotation (Line(
            points={{-45,82},{-28,82},{-28,80},{-13,80},{-13,59}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(flow1Dim.Wall_int, source_T.ThermalPortCell) annotation (Line(
            points={{-14,23},{-14,50.9},{-13.1,50.9}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(sourceMdot1.flangeB, flow1Dim.InFlow) annotation (Line(
            points={{-71,22},{-40,22},{-40,18},{-24,18}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, sinkP.flangeB) annotation (Line(
            points={{-4,18.1},{16,18.1},{16,16},{41.6,16},{41.6,14}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sine.y, sourceMdot1.in_Mdot) annotation (Line(
            points={{-99.3,51},{-86,51},{-86,28}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{
                  100,100}}), graphics),
          experiment(StopTime=50),
          __Dymola_experimentSetupOutput);
      end Cell1D_Trunc;

      model flow1D_Trunc
        parameter Integer N = 10;
        ThermoCycle.Components.FluidFlow.Pipes.Flow1Dim flow1Dim(
          A=2,
          Unom_l=400,
          Unom_tp=1000,
          Unom_v=400,
          N=N,
          V=0.003,
          Mdotnom=0.3,
          Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          redeclare model Flow1DimHeatTransferModel =
              ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.Constant,
          pstart=1000000,
          Tstart_inlet=323.15,
          Tstart_outlet=373.15,
          Mdotconst=true)
          annotation (Placement(transformation(extent={{-40,-4},{-2,34}})));

        ThermoCycle.Components.HeatFlow.Sources.Source_T source_T(N=N)
          annotation (Placement(transformation(extent={{-30,42},{4,64}})));
        Modelica.Blocks.Sources.Constant const(k=273.15 + 140)
          annotation (Placement(transformation(extent={{-62,64},{-54,72}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          Mdot_0=0.3335,
          UseT=false,
          h_0=84867,
          p=888343,
          T_0=356.26)
          annotation (Placement(transformation(extent={{-86,12},{-60,38}})));
        ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP(
          redeclare package Medium = CoolProp2Modelica.Media.SES36_CP,
          h=254381,
          p0=866735)
          annotation (Placement(transformation(extent={{50,14},{70,34}})));
        Modelica.Blocks.Sources.Sine sine(
          startTime=10,
          offset=0.3335,
          amplitude=0.5,
          phase=0,
          freqHz=0.1)
          annotation (Placement(transformation(extent={{-126,40},{-112,54}})));
      equation
        connect(source_T.thermalPort, flow1Dim.Wall_int) annotation (Line(
            points={{-13.17,48.49},{-13.17,43.95},{-21,43.95},{-21,22.9167}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(const.y, source_T.Temperature) annotation (Line(
            points={{-53.6,68},{-13,68},{-13,57.4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sourceMdot1.flangeB, flow1Dim.InFlow) annotation (Line(
            points={{-61.3,25},{-52,25},{-52,15},{-36.8333,15}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(flow1Dim.OutFlow, sinkP.flangeB) annotation (Line(
            points={{-5.16667,15.1583},{18,15.1583},{18,24},{51.6,24}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sine.y, sourceMdot1.in_Mdot) annotation (Line(
            points={{-111.3,47},{-97.65,47},{-97.65,32.8},{-80.8,32.8}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{
                  100,100}}), graphics={Text(
                extent={{-62,56},{-26,50}},
                lineColor={0,0,0},
                textString="Thermal port")}),
          experiment(StopTime=50, __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput);
      end flow1D_Trunc;
    end TestNumericalMethods;
  annotation (Documentation(info="<HTML>
<p><big><dl><dt><b>Main Authors:</b> <br/></dt>
<dd>Sylvain Quoilin; &LT;<a href=\"squoilin@ulg.ac.be\">squoilin@ulg.ac.be</a>&GT;</dd>
<dd>Adriano Desideri &LT;<a href=\"adesideri@ulg.ac.be\">adesideri@ulg.ac.be</a>&GT;<br/></dd>
<dd>University of Liege</dd>
<dd>Laboratory of thermodynamics</dd>
<dd>Campus du Sart-Tilman Bât B49 (P33)</dd>
<dd>B-4000 Liège - BELGIUM -<br/></dd>
<dt><b>Copyright:</b> </dt>
<dd>Copyright &copy; 2013-2014, Sylvain Quoilin and Adriano Desideri.<br/></dd>
<dd><i>The IndustrialControlSystems package is <b>free</b> software; it can be redistributed and/or modified under the terms of the <b>Modelica license</b>.</i><br/></dd>
</dl></html>"));
  end Examples;

  model Building
   AixLib.Building.LowOrder.Multizone.Multizone multizone(
        buildingParam=Project.ResidentialBuilding.ResidentialBuilding_DataBase.ResidentialBuilding_base(),
     redeclare AixLib.Building.LowOrder.ThermalZone.ThermalZoneEquipped zone(
     redeclare
          AixLib.Building.LowOrder.BaseClasses.BuildingPhysics.BuildingPhysics
          buildingPhysics(
     redeclare
            AixLib.Building.Components.WindowsDoors.BaseClasses.CorrectionSolarGain.CorG_VDI6007
            corG)))
      annotation (Placement(transformation(extent={{-4,240},{50,290}})));
    AixLib.Building.Components.Weather.Weather weather(
      Outopt=1,
      Air_temp=true,
      Mass_frac=true,
      Sky_rad=true,
      Ter_rad=true,
      fileName=Modelica.Utilities.Files.loadResource(
        "modelica://AixLib/Resources/WeatherData/TRY2010_12_Jahr_Modelica-Library.txt"),
      tableName="wetter",
        Latitude=49.5,
        Longitude=8.5,
      SOD=AixLib.DataBase.Weather.SurfaceOrientation.SurfaceOrientationBaseDataDefinition(
       nSurfaces=5,
      name={"0.0", "90.0", "180.0", "270.0", "-1"},
      Azimut={180.0, -90.0, 0.0, 90.0, 0.0},
      Tilt={90.0, 90.0, 90.0, 90.0, 0.0}))
      annotation (Placement(transformation(extent={{-4,322},{26,342}})));
    AixLib.Utilities.HeatTransfer.HeatToStar
                                      HeatTorStar(A = 2) annotation(Placement(transformation(extent={{-88,200},
              {-68,220}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow machinesConvective annotation(Placement(transformation(extent={{-124,
              248},{-104,268}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow personsConvective annotation(Placement(transformation(extent={{-124,
              228},{-104,248}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow personsRadiative annotation(Placement(transformation(extent={{-124,
              200},{-104,220}})));
    Modelica.Blocks.Sources.Constant const(k=200)
      annotation (Placement(transformation(extent={{-204,230},{-184,250}})));
    Modelica.Blocks.Math.Gain gain(k=0.4)
      annotation (Placement(transformation(extent={{-162,254},{-152,264}})));
    Modelica.Blocks.Math.Gain gain1(k=0.2)
      annotation (Placement(transformation(extent={{-160,234},{-150,244}})));
    Modelica.Blocks.Math.Gain gain2(k=0.4)
      annotation (Placement(transformation(extent={{-160,206},{-152,214}})));
  equation
    connect(weather.SolarRadiation_OrientedSurfaces,multizone. radIn)
      annotation (Line(points={{3.2,321},{3.2,304.5},{6.26,304.5},{6.26,287.5}},
          color={255,128,0}));
    connect(weather.WeatherDataVector,multizone. weather) annotation (
       Line(points={{10.9,321},{10.9,306},{18.68,306},{18.68,288.5}},
                                                                  color={0,0,127}));
    connect(personsRadiative.port,HeatTorStar. Therm) annotation(Line(points={{-104,
            210},{-87.2,210}},                                                                           color = {191, 0, 0}));
    connect(machinesConvective.Q_flow,gain. y) annotation (Line(
        points={{-124,258},{-136,258},{-136,259},{-151.5,259}},
        color={0,0,127}));
    connect(gain2.y,personsRadiative. Q_flow) annotation (Line(
        points={{-151.6,210},{-124,210}},
        color={0,0,127}));
    connect(gain1.y,personsConvective. Q_flow) annotation (Line(
        points={{-149.5,239},{-134,239},{-134,238},{-124,238}},
        color={0,0,127}));
    connect(const.y,gain. u) annotation (Line(
        points={{-183,240},{-172,240},{-172,259},{-163,259}},
        color={0,0,127}));
    connect(const.y,gain1. u) annotation (Line(
        points={{-183,240},{-176,240},{-176,239},{-161,239}},
        color={0,0,127}));
    connect(const.y,gain2. u) annotation (Line(
        points={{-183,240},{-176,240},{-176,210},{-160.8,210}},
        color={0,0,127}));
    connect(HeatTorStar.Star,multizone. internalGainsRad[1]) annotation (Line(points={{-68.9,
            210},{36,210},{36,220},{36,242},{36,242.5},{36.5,242.5}},
                                                        color={95,95,95}));
    connect(machinesConvective.port,multizone. internalGainsConv[1])
      annotation (Line(points={{-104,258},{-40,258},{-40,242.5},{27.32,242.5}}, color={191,0,0}));
    connect(personsConvective.port,multizone. internalGainsConv[1])
      annotation (Line(points={{-104,238},{-40,238},{-40,242.5},{27.32,242.5}}, color={191,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Building;

  model TestExchanger

    ThermoCycle.Components.Units.HeatExchangers.Hx1DInc condenser(
      redeclare package Medium1 = ThermoCycle.Media.Air_CP,
      redeclare package Medium2 = ThermoCycle.Media.StandardWater,
      N=10,
      redeclare model Medium1HeatTransferModel =
          ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.MassFlowDependence,
      M_wall=5,
      Mdotnom_sf=0.5,
      Mdotnom_wf=0.1,
      A_sf=4,
      A_wf=2,
      Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
      V_sf=0.01,
      V_wf=0.001,
      Unom_sf=3000,
      steadystate_h_wf=false,
      pstart_wf=380000,
      Tstart_inlet_wf=345.15,
      Tstart_outlet_wf=310.15,
      Tstart_inlet_sf=310.15,
      Tstart_outlet_sf=310.15)
      annotation (Placement(transformation(extent={{10,-4},{-16,22}})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot(
        redeclare package Medium = ThermoCycle.Media.StandardWater,
      Mdot_0=1.76,
      T_0=283.15)
      annotation (Placement(transformation(extent={{-72,12},{-52,32}})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SourceP sourceP(redeclare
        package                                                                   Medium =
                 ThermoCycle.Media.StandardWater)
     annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={52,28})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SourceP sourceP1(redeclare
        package                                                                    Medium =
                 ThermoCycle.Media.Air_CP)
      annotation (Placement(transformation(extent={{-76,-24},{-56,-4}})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot sourceMdot1(
        redeclare package Medium = ThermoCycle.Media.Air_CP,
      Mdot_0=1.76,
      T_0=343.15) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={52,-16})));
  equation
    connect(sourceMdot.flangeB, condenser.inlet_fl2)
      annotation (Line(points={{-53,22},{-32,22},{-32,15},{-12.8,15}}, color={0,0,255}));
    connect(condenser.outlet_fl2, sourceP.flange)
      annotation (Line(points={{6.8,14.8},{24.4,14.8},{24.4,28},{42.6,28}}, color={0,0,255}));
    connect(condenser.outlet_fl1, sourceP1.flange)
      annotation (Line(points={{-13,4},{-34,4},{-34,-14},{-56.6,-14}}, color={0,0,255}));
    connect(sourceMdot1.flangeB, condenser.inlet_fl1)
      annotation (Line(points={{43,-16},{26,-16},{26,4},{7,4}}, color={0,0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end TestExchanger;

  model heatpump

    import Project.*;
    import Modelica.*;
    import Modelica.Blocks.Interfaces.*;

    ThermoCycle.Components.Units.HeatExchangers.Hx1DInc condenser(
      redeclare package Medium1 = ThermoCycle.Media.R134a_CP,
      redeclare package Medium2 = ThermoCycle.Media.StandardWater,
      N=10,
      redeclare model Medium1HeatTransferModel =
          ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.MassFlowDependence,
      M_wall=5,
      Mdotnom_sf=0.5,
      Mdotnom_wf=0.1,
      A_sf=4,
      A_wf=2,
      Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
      V_sf=0.01,
      V_wf=0.001,
      Unom_sf=3000,
      steadystate_h_wf=false,
      pstart_wf=380000,
      Tstart_inlet_wf=345.15,
      Tstart_outlet_wf=310.15,
      Tstart_inlet_sf=310.15,
      Tstart_outlet_sf=310.15)
      annotation (Placement(transformation(extent={{10,16},{-16,42}})));

    ThermoCycle.Components.Units.HeatExchangers.Hx1DInc evaporator(
      redeclare package Medium1 = ThermoCycle.Media.R134a_CP,
      N=10,
      redeclare model Medium1HeatTransferModel =
          ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.MassFlowDependence,
      M_wall=10,
      Mdotnom_wf=0.1,
      A_wf=4,
      Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
      V_sf=0.002,
      V_wf=0.002,
      redeclare package Medium2 = Modelica.Media.Air.DryAirNasa,
      A_sf=20,
      Unom_sf=100,
      Mdotnom_sf=0.76,
      steadystate_h_wf=false,
      pstart_wf=230000,
      Tstart_inlet_wf=263.15,
      Tstart_outlet_wf=277.15,
      Tstart_inlet_sf=283.15,
      Tstart_outlet_sf=275.15)
      annotation (Placement(transformation(extent={{-12,-44},{14,-70}})));

    ThermoCycle.Components.Units.PdropAndValves.Valve valve(
      redeclare package Medium = ThermoCycle.Media.R134a_CP,
      Mdot_nom=0.1,
      UseNom=false,
      Afull=15e-7,
      Xopen=0.45,
      p_nom=380000,
      T_nom=308.15,
      DELTAp_nom=150000)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-40,-26})));

     ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor
                                                              compressor(
      epsilon_v=0.9,
      redeclare package Medium = ThermoCycle.Media.R134a_CP,
      V_s=8e-4,
      p_su_start=200000,
      p_ex_start=400000) annotation (Placement(transformation(
          extent={{-19,-18},{19,18}},
          rotation=180,
          origin={59,-16})));

    ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot airSideSource(
      redeclare package Medium = Modelica.Media.Air.DryAirNasa,
      Mdot_0=1.76,
      T_0=283.15)
      annotation (Placement(transformation(extent={{66,-88},{46,-68}})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SinkP airSideSink(redeclare
        package Medium =
                 Modelica.Media.Air.DryAirNasa)
      annotation (Placement(transformation(extent={{-52,-74},{-72,-54}})));

    ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                                 electricDrive
      annotation (Placement(transformation(extent={{28,-26},{8,-6}})));

    ThermoCycle.Components.Units.Tanks.ThermoclineStorage thermoclineStorage(
        replaceable package Medium = ThermoCycle.Media.StandardWater,
        N=10,
        V_tank=20,
        H_D=2.5,
        d_met=0.03,
        epsilon_p=1,
        Vlstart=15,
        p=101000,
        k_liq=500,
        k_wall=200,
        U_env_bottom=200,
        U_env_wall=200,
        U_env_top=200,
        pstart=101000,
        m_dot_su=0.02,
        m_dot_ex=0.02,
        Tstart_su=320,
        Tstart_ex=330)
       annotation (Placement(transformation(extent={{-8,-10},{8,10}},
          rotation=180,
          origin={-4,82})));

    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTpofEvaporator(redeclare
        package Medium =
                 ThermoCycle.Media.R134a_CP)
      annotation (Placement(transformation(extent={{32,-58},{52,-38}})));

    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTpofCompressor(redeclare
        package Medium =
                 ThermoCycle.Media.R134a_CP) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={50,16})));
    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTpofCondenser(redeclare
        package Medium =
                 ThermoCycle.Media.R134a_CP) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-45,6})));
    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTpofHeatedWater(redeclare
        package Medium =
                 ThermoCycle.Media.StandardWater)
      annotation (Placement(transformation(extent={{24,30},{44,50}})));

    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensTpofTank(redeclare
        package Medium =
                 ThermoCycle.Media.StandardWater)
      annotation (Placement(transformation(extent={{10,80},{30,100}})));
    ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump pumpTankWater(
      redeclare package Medium = ThermoCycle.Media.StandardWater,
      PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.ORCNext,
      PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.freq,
      hstart=2.27e5,
      M_dot_start=0.1,
      eta_is=0.9,
      epsilon_v=0.9,
      V_dot_max=0.05)
        annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={-40,56})));

    Modelica.Blocks.Sources.Step step(
      offset=50,
      height=5,
      startTime=50) annotation (Placement(transformation(extent={{-12,-2},{-4,6}})));

    ThermoCycle.Components.FluidFlow.Sensors.SensMdot sensMdot(
    replaceable package Medium = ThermoCycle.Media.StandardWater)
      annotation (Placement(transformation(extent={{40,80},{60,100}})));

    ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Pump pumpAir(
      redeclare package Medium = ThermoCycle.Media.StandardWater,
      PumpType=ThermoCycle.Functions.Enumerations.PumpTypes.UD,
      PumpInput=ThermoCycle.Functions.Enumerations.PumpInputs.freq,
      hstart=2.27e5,
      M_dot_start=0.1,
      eta_is=0.9,
      epsilon_v=0.9,
      V_dot_max=0.05)
        annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={-50,118})));
    ThermoCycle.Components.Units.HeatExchangers.Hx1DInc evaporator1(
      redeclare package Medium1 = ThermoCycle.Media.Air_CP,
      N=10,
      redeclare model Medium1HeatTransferModel =
          ThermoCycle.Components.HeatFlow.HeatTransfer.ConvectiveHeatTransfer.MassFlowDependence,
      M_wall=10,
      Mdotnom_wf=0.1,
      A_wf=4,
      Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind_AllowFlowReversal,
      V_sf=0.002,
      V_wf=0.002,
      redeclare package Medium2 = ThermoCycle.Media.StandardWater,
      A_sf=20,
      Unom_sf=100,
      Mdotnom_sf=0.76,
      steadystate_h_wf=false,
      pstart_wf=230000,
      Tstart_inlet_wf=263.15,
      Tstart_outlet_wf=277.15,
      Tstart_inlet_sf=283.15,
      Tstart_outlet_sf=275.15)
      annotation (Placement(transformation(extent={{-14,160},{12,134}})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot airSideSource1(
      redeclare package Medium = ThermoCycle.Media.Air_CP,
      Mdot_0=1.76,
      T_0=283.15)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=180,
          origin={-58,154})));
    ThermoCycle.Components.FluidFlow.Reservoirs.SinkP airSideSink1(
                                                                  redeclare
        package Medium =
                 ThermoCycle.Media.Air_CP)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=180,
          origin={58,162})));
   AixLib.Building.LowOrder.Multizone.Multizone multizone(
        buildingParam=Project.ResidentialBuilding.ResidentialBuilding_DataBase.ResidentialBuilding_base(),
     redeclare AixLib.Building.LowOrder.ThermalZone.ThermalZoneEquipped zone(
     redeclare
          AixLib.Building.LowOrder.BaseClasses.BuildingPhysics.BuildingPhysics
          buildingPhysics(
     redeclare
            AixLib.Building.Components.WindowsDoors.BaseClasses.CorrectionSolarGain.CorG_VDI6007
            corG)))
      annotation (Placement(transformation(extent={{20,264},{74,314}})));
    AixLib.Building.Components.Weather.Weather weather(
      Outopt=1,
      Air_temp=true,
      Mass_frac=true,
      Sky_rad=true,
      Ter_rad=true,
      fileName=Modelica.Utilities.Files.loadResource(
        "modelica://AixLib/Resources/WeatherData/TRY2010_12_Jahr_Modelica-Library.txt"),
      tableName="wetter",
        Latitude=49.5,
        Longitude=8.5,
      SOD=AixLib.DataBase.Weather.SurfaceOrientation.SurfaceOrientationBaseDataDefinition(
       nSurfaces=5,
      name={"0.0", "90.0", "180.0", "270.0", "-1"},
      Azimut={180.0, -90.0, 0.0, 90.0, 0.0},
      Tilt={90.0, 90.0, 90.0, 90.0, 0.0}))
      annotation (Placement(transformation(extent={{20,346},{50,366}})));
    AixLib.Utilities.HeatTransfer.HeatToStar
                                      HeatTorStar(A = 2) annotation(Placement(transformation(extent={{-64,224},
              {-44,244}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow machinesConvective annotation(Placement(transformation(extent={{-100,
              272},{-80,292}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow personsConvective annotation(Placement(transformation(extent={{-100,
              252},{-80,272}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow personsRadiative annotation(Placement(transformation(extent={{-100,
              224},{-80,244}})));
    Modelica.Blocks.Sources.Constant const(k=200)
      annotation (Placement(transformation(extent={{-180,254},{-160,274}})));
    Modelica.Blocks.Math.Gain gain(k=0.4)
      annotation (Placement(transformation(extent={{-138,278},{-128,288}})));
    Modelica.Blocks.Math.Gain gain1(k=0.2)
      annotation (Placement(transformation(extent={{-136,258},{-126,268}})));
    Modelica.Blocks.Math.Gain gain2(k=0.4)
      annotation (Placement(transformation(extent={{-136,230},{-128,238}})));

    Modelica.Blocks.Sources.CombiTimeTable tableInternalGains(
      tableOnFile=true,
      extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
      tableName="Internals",
      columns=2:4,
      fileName=Modelica.Utilities.Files.loadResource(
          "modelica://Project/ResidentialBuilding//InternalGains_ResidentialBuilding.mat"))
      annotation (Placement(transformation(extent={{126,296},{106,316}})));

    ThermoCycle.Components.HeatFlow.Sensors.SensTp sensofAir(redeclare package
        Medium = ThermoCycle.Media.Air_CP)
      annotation (Placement(transformation(extent={{20,166},{40,186}})));

    RealOutput M_air;
    RealOutput T_air;

  equation

    M_air = evaporator1.outlet_fl1.m_flow;
    T_air = sensofAir.T;

    connect(evaporator.inlet_fl1, valve.OutFlow) annotation (Line(
        points={{-9,-52},{-40,-52},{-40,-35}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(airSideSource.flangeB, evaporator.inlet_fl2) annotation (Line(
        points={{47,-78},{20,-78},{20,-63},{10.8,-63}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(evaporator.outlet_fl2, airSideSink.flangeB) annotation (Line(
        points={{-8.8,-62.8},{-20,-62.8},{-20,-64},{-53.6,-64}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(electricDrive.shaft, compressor.flange_elc) annotation (Line(
        points={{26.6,-16},{46.3333,-16}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(evaporator.outlet_fl1, sensTpofEvaporator.InFlow) annotation (Line(
        points={{11,-52},{35,-52},{35,-52.8}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(sensTpofEvaporator.OutFlow, compressor.InFlow) annotation (Line(
          points={{49,-52.8},{69.7667,-52.8},{69.7667,-27.7}}, color={0,0,255}));
    connect(compressor.OutFlow, sensTpofCompressor.OutFlow) annotation (Line(
          points={{45.3833,-10},{45.2,-10},{45.2,9}}, color={0,0,255}));
    connect(sensTpofCompressor.InFlow, condenser.inlet_fl1) annotation (Line(
          points={{45.2,23},{30.5,23},{30.5,24},{7,24}}, color={0,0,255}));
    connect(condenser.outlet_fl1, sensTpofCondenser.OutFlow) annotation (Line(
          points={{-13,24},{-28,24},{-40.2,24},{-40.2,13}}, color={0,0,255}));
    connect(sensTpofCondenser.InFlow, valve.InFlow) annotation (Line(points={{-40.2,
            -1},{-40.2,-8.5},{-40,-8.5},{-40,-17}}, color={0,0,255}));
    connect(condenser.outlet_fl2, sensTpofHeatedWater.InFlow) annotation (Line(
          points={{6.8,34.8},{16.4,34.8},{16.4,35.2},{27,35.2}}, color={0,0,255}));
    connect(sensTpofHeatedWater.OutFlow, thermoclineStorage.portHotSF)
      annotation (Line(points={{41,35.2},{64,35.2},{64,75.9},{3.5,75.9}}, color={0,
            0,255}));
    connect(thermoclineStorage.portColdSF, sensTpofTank.InFlow) annotation (Line(
          points={{3.4,88.4},{8.7,88.4},{8.7,85.2},{13,85.2}}, color={0,0,255}));
    connect(thermoclineStorage.portHotPW, pumpTankWater.InFlow)
      annotation (Line(points={{-11.4,76},{-39.5,76},{-39.5,63.2}}, color={0,0,255}));
    connect(pumpTankWater.OutFlow, condenser.inlet_fl2)
      annotation (Line(points={{-32.6,50.4},{-32.6,35},{-12.8,35}}, color={0,0,255}));
    connect(step.y, electricDrive.f)
      annotation (Line(points={{-3.6,2},{-3.6,2},{17.6,2},{17.6,-6.6}}, color={0,0,127}));
    connect(sensTpofTank.OutFlow, sensMdot.InFlow)
      annotation (Line(points={{27,85.2},{29,85.2},{29,86},{46,86}},     color={0,0,255}));
    connect(pumpAir.OutFlow, thermoclineStorage.portColdPW) annotation (Line(points={{-42.6,
            112.4},{-42.6,100.2},{-11.7,100.2},{-11.7,88.35}},
                                                 color={0,0,255}));
    connect(sensMdot.OutFlow, evaporator1.inlet_fl2)
      annotation (Line(points={{54,86},{70,86},{70,141},{8.8,141}}, color={0,0,255}));
    connect(evaporator1.outlet_fl2, pumpAir.InFlow)
      annotation (Line(points={{-10.8,141.2},{-49.5,141.2},{-49.5,125.2}}, color={0,0,255}));
    connect(airSideSource1.flangeB, evaporator1.inlet_fl1)
      annotation (Line(points={{-49,154},{-30,154},{-30,152},{-11,152}}, color={0,0,255}));
    connect(weather.SolarRadiation_OrientedSurfaces,multizone. radIn)
      annotation (Line(points={{27.2,345},{27.2,328.5},{30.26,328.5},{30.26,311.5}},
          color={255,128,0}));
    connect(weather.WeatherDataVector,multizone. weather) annotation (
       Line(points={{34.9,345},{34.9,330},{42.68,330},{42.68,312.5}},
                                                                  color={0,0,127}));
    connect(personsRadiative.port,HeatTorStar. Therm) annotation(Line(points={{-80,234},{-63.2,234}},    color = {191, 0, 0}));
    connect(machinesConvective.Q_flow,gain. y) annotation (Line(
        points={{-100,282},{-112,282},{-112,283},{-127.5,283}},
        color={0,0,127}));
    connect(gain2.y,personsRadiative. Q_flow) annotation (Line(
        points={{-127.6,234},{-100,234}},
        color={0,0,127}));
    connect(gain1.y,personsConvective. Q_flow) annotation (Line(
        points={{-125.5,263},{-110,263},{-110,262},{-100,262}},
        color={0,0,127}));
    connect(const.y,gain. u) annotation (Line(
        points={{-159,264},{-148,264},{-148,283},{-139,283}},
        color={0,0,127}));
    connect(const.y,gain1. u) annotation (Line(
        points={{-159,264},{-152,264},{-152,263},{-137,263}},
        color={0,0,127}));
    connect(const.y,gain2. u) annotation (Line(
        points={{-159,264},{-152,264},{-152,234},{-136.8,234}},
        color={0,0,127}));
    connect(HeatTorStar.Star,multizone. internalGainsRad[1]) annotation (Line(points={{-44.9,234},{60,234},
            {60,244},{60,266},{60,266.5},{60.5,266.5}}, color={95,95,95}));
    connect(machinesConvective.port, multizone.internalGainsConv[1])
      annotation (Line(points={{-80,282},{-16,282},{-16,266.5},{51.32,266.5}}, color={191,0,0}));
    connect(personsConvective.port, multizone.internalGainsConv[1])
      annotation (Line(points={{-80,262},{-16,262},{-16,266.5},{51.32,266.5}}, color={191,0,0}));
    connect(tableInternalGains.y[1], multizone.internalGains[1])
      annotation (Line(points={{105,306},{72.11,306},{72.11,305.25}}, color={0,0,127}));

    connect(evaporator1.outlet_fl1, sensofAir.InFlow)
      annotation (Line(points={{9,152},{16,152},{16,171.2},{23,171.2}}, color={0,0,255}));
    connect(sensofAir.OutFlow, airSideSink1.flangeB)
      annotation (Line(points={{37,171.2},{43.5,171.2},{43.5,162},{49.6,162}}, color={0,0,255}));

    connect(M_air,multizone.ventilationTemperature[1]);
    connect(T_air,multizone.ventilationRate[1]);

   annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-240,-100},{220,360}})),
      experiment(StopTime=200),
      __Dymola_experimentSetupOutput,
      Icon(coordinateSystem(extent={{-240,-100},{220,360}})));
  end heatpump;
  annotation (uses(Modelica(version="3.2.2"),
      AixLib(version="0.3.2"),
      Project(version="1")));
end HeatPumpTest;
