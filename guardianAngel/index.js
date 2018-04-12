var express = require('express')
var app = express()
var bodyParser = require('body-parser')
var fs = require('fs');
 
app.use(bodyParser.json())
app.use(bodyParser.urlencoded({extended: false}))
 
app.listen(3000)

console.log("hellllllloooooooo");
var userCoorFilePath;
var rlogger;
var roverCoorFilePath;
var lastlat;
var lastlon;
var lastbearing;

app.post('/guardian-angel', function(req, res) {


  //console.log(res);

  var mtype = req.body.message_type;
  var name = req.body.user_name;
  var logger;
  //var userCoorFilePath;
  var lat;
  var lon;
  var tripID;
  
  var buffer;
  var buffer2;
  var ended;
  var homelat;
  var homelon;

  console.log(mtype);

  const db = require('./db.js');
  if (mtype == 1) {
    lat = req.body.lat;
    lon = req.body.lon;
    db.query("SELECT tripID FROM Trips WHERE gotHome = false", function(err,results,fields){
      if(err) console.log(err);
      if(results.length > 0){
        console.log("found tripID");
        buffer = lat + ", " + lon + "\n";
        fs.appendFile(userCoorFilePath, buffer, function (err) {
          if (err) throw err;
          console.log('Saved!');
        });
      } else {
        console.log("did not find tripID");
        db.query("INSERT INTO Trips (student) Values (?)", [name], function(err, results, fields){
          if(err) console.log(err);
          db.query("SELECT tripID FROM Trips WHERE gotHome = false", function(err,results,fields){
            tripID = results[0].tripID;
            roverCoorFilePath = "coordinates/rover/trip"+ tripID + ".txt";
            db.query("INSERT INTO Rover (tripID, location) Values (?, ?)", [tripID, roverCoorFilePath], function(err, results, fields) {
              if(err) console.log(err);
            });
            userCoorFilePath = "coordinates/user/" + name  + tripID + ".txt";//name  + tripID + ".txt";//
            db.query("UPDATE Trips SET location = ? WHERE tripID = ?", [userCoorFilePath, tripID], function(err, results, fields){
              if(err) console.log(err);
              fs.writeFile(userCoorFilePath, lat + ', ' + lon + '\n', function (err) {
                if (err) throw err;
                console.log('It\'s saved! in same location.');
              });
            });
            db.query("UPDATE Trips SET lat = ?, lon = ? WHERE tripID = ?", [lat, lon, tripID], function(err, results, fields){ 
              if(err) console.log(err);
            });
          });
        });
      }
    });
  } else if (mtype == 2) {
    console.log("mtype == 2\n");
    db.query("UPDATE Trips SET endT = NOT endT WHERE endT = false", function(err, results, fields){
      console.log("End trip!\n");
      if(err) console.log(err);
    });
  } else if (mtype == 3) {
    console.log("Emergency!\n");
    db.query("UPDATE Trips SET emergency = NOT emergency WHERE endT = false", function(err, results, fields){
      if(err) console.log(err);
    });
  } else if (mtype == 4) {
    console.log("arrived!\n");
    db.query("UPDATE Trips SET arrived = NOT arrived WHERE endT = false", function(err, results, fields){
      if(err) console.log(err);
    });
  } else if (mtype == 5) {
    db.query("UPDATE rover SET direction = ?, length = ? WHERE tripID = ?", [req.body.direction, req.body.length, tripID], function(err, results, fields){
      if(err) console.log(err);
    });
  } else if (mtype == 0) {
    lat = req.body.lat;
    lon = req.body.lon;
    bearing = req.body.bearing;
    db.query("SELECT tripID, endT FROM Trips WHERE gotHome = false", function(err,results,fields){
      if(results.length > 0) {
        tripID = results[0].tripID;
        ended = results[0].endT;
        db.query("SELECT location FROM Rover WHERE tripID = ?", [tripID], function(err,results,fields){
          if(results.length > 0){
            buffer2 = lat + ", " + lon + ", " + bearing + "\n";
            console.log(bearing);
            fs.appendFile(roverCoorFilePath, buffer2, function (err) {
              if (err) throw err;
              console.log('Saved!');
            });
            if(ended == 1) {
              db.query("SELECT lat, lon FROM trips WHERE gotHome = false"), function(err,results,fields) {
                homelat = results[0].lat;
                homelon = results[0].lon;
                if (Math.abs(homelat - lat) <= 1 && Math.abs(homelon-lon) <= 1) {
                  db.query("UPDATE Trips SET gotHome = NOT gotHome WHERE gotHome = false", function(err, results, fields){
                    if(err) console.log(err);
                  });
                }
              }
            }
          }
        });
        db.query("UPDATE rover SET bearing = ? WHERE tripID = ?", [bearing, tripID], function(err,results,fields){ 
          if(err) console.log(err);
        });
      }
    });
  }
  res.end();
});

app.get('/rover', function(req, res) {
  console.log("Found rover");
  
  const db = require('./db.js');
  var tripID;
  var jsonobj = {"emergency": 0, "arrived": 0, "ended": 0, "bearing": 0, "direction": 0, "length": 0}
 
  
  db.query("SELECT tripID FROM Trips WHERE gotHome = false", function(err,results,fields){
    if(err) console.log(err);
    if(results.length > 0) {
      tripID = results[0].tripID;
      db.query("SELECT emergency FROM Trips WHERE tripID = ?", [tripID], function(err,results,fields){
        if(err) console.log(err);
        jsonobj.emergency = results[0].emergency;
        db.query("SELECT arrived FROM Trips WHERE tripID = ?", [tripID], function(err,results,fields){
          if(err) console.log(err);
          jsonobj.arrived = results[0].arrived;
          db.query("SELECT endT FROM Trips WHERE tripID = ?", [tripID], function(err,results,fields){
            if(err) console.log(err);
            jsonobj.endT = results[0].endT;
            db.query("SELECT bearing FROM rover WHERE tripID = ?", [tripID], function(err,results,fields){
              if(err) console.log(err);
              if(results.length > 0) {
                jsonobj.bearing = results[0].bearing;
                db.query("SELECT distance FROM rover WHERE tripID = ?", [tripID], function(err,results,fields){
                  if(err) console.log(err);
                  if(results[0].distance != null) {
                    jsonobj.direction = results[0].distance;
                  }
                  db.query("SELECT len FROM rover WHERE tripID = ?", [tripID], function(err,results,fields){
                    if(err) console.log(err);
                    if(results[0].distance != null) {
                      jsonobj.len = results[0].len;
                    }
                    res.setHeader('Content-Type', 'application/json');
                    res.send(JSON.stringify(jsonobj, null, 2));
                  });
                });
              }
            });
          });
        });
      });
    }
  });
});


app.get('/usercoord' , function(req, res) {
  console.log("Ask for coordinates");
  const db = require('./db.js');
  var tripID;
  var userc = {};

  db.query("SELECT tripID FROM Trips WHERE endT = false", function(err,results,fields){
    if(err) console.log(err);
    if(results.length > 0) {
      tripID = results[0].tripID;
      db.query("SELECT lat, lon FROM Trips WHERE tripID = ?", [tripID], function(err,results,fields){
        if(err) console.log(err);
        if(results.length > 0) {
          userc.lat = results[0].lat;
          userc.lon = results[0].lon;
          res.setHeader('Content-Type', 'application/json');
          res.send(JSON.stringify(userc, null, 2));
        }
      });
    }
  });
});
