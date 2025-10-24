import adsk.core, adsk.fusion, traceback
import csv 
import time
import os

def run(context):
    ui = None
    dir = "C:\\Users\\vitaminmoo\\repos\\aioli-foc\\mechanical\\motor_parametric\\"
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = app.activeProduct
        rootComp = design.rootComponent

        screwTable = {
            # motorScrewHoleDiameter, motorScrewHeadHeight, motorScrewHeadDiameter
            '1.6': (1.8, 1.6, 3.14),
            '2.0': (2.4, 2.0, 3.98),
            '2.5': (2.9, 2.5, 4.68),
        }

        with open(os.path.join(dir, 'motors.csv'), mode='r') as motorsFile:
            reader = csv.DictReader(motorsFile)
            for row in reader:
                for key in row:
                    if key == 'motorScrewM':
                        i = 0
                        for a in ('motorScrewHoleDiameter', 'motorScrewHeadHeight', 'motorScrewHeadDiameter'):
                            p = design.userParameters.itemByName(a)
                            value = screwTable[row['motorScrewM']][i]
                            p.expression = str(value)
                            i += 1
                    if key == 'motorMagnetDiameter':
                        row['motorMagnetDiameter'] = str(max(float(row['motorMagnetDiameter']), 13)) # arbitrary minimum
                    if key == 'motorMagnetLength':
                        row['motorMagnetLength'] = str(max(float(row['motorMagnetLength']), 3.5)) # clear a molex connector

                    p = design.userParameters.itemByName(key)
                    try:
                        float(row[key])
                        p.expression = row[key]
                    except:
                        p.comment = row[key]
                    
                exportMgr = design.exportManager

                allBodies = rootComp.bRepBodies
                for body in allBodies:
                    filename = os.path.join(dir, 'stls', row['motorModel'] + '_' + body.name + '.stl')
                    stlExportOptions = exportMgr.createSTLExportOptions(body, filename)
                    stlExportOptions.sendToPrintUtility = False
                    exportMgr.execute(stlExportOptions)
            ui.messageBox('Complete')
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))