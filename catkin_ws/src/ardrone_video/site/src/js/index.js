var Settings = Backbone.Model.extend({

  defaults: {
    signaling_server: {
        url: 'ws://127.0.0.1:9000'
    },
    ice_servers: [{
        url: navigator.mozGetUserMedia ? 'stun:stun.services.mozilla.com' :
             navigator.webkitGetUserMedia ? 'stun:stun.l.google.com:19302' :
             'stun:23.21.150.121'
    }],
  },

  initialize: function (attributes, options) {
    options = options || {};
    _.defaults(options , {
        storage: window.localStorage
    });
    this.storage = options.storage;
  },

  fetch: function () {
    _.each(_.keys(this.attributes), function (key) {
      if (this.storage.getItem(key) !== null) {
        this.set(key, JSON.parse(this.storage.getItem(key)));
      } else if (key in this.defaults){
        this.set(key, this.defaults[key]);
      } else {
        this.unset(key);
      }
    }, this);
  },

  reset: function () {
    _.each(
        this.defaults,
        function (value, key) { this.set(key, value); },
        this
    );
  },

  save: function () {
    _.each(
        this.attributes,
        function (v, k) {
            this.storage.setItem(k, v === null ? null : JSON.stringify(v));
        },
        this
    );
  }

});

var SettingsDialog = Backbone.View.extend({

  initialize: function (options) {
      var that = this;

      options = options || {};
      this.settings = options.settings;

      this.$el.find('#settings-toggle').on('click', function () {
          that.toggle();
      });

      this.$el.find('#settings-defaults').on('click', function () {
          that.settings.reset();
          that.load();
      });

      this.$el.find('#settings-save').on('click', $.proxy(function () {
          this.save();
          this.$el.find('#settings').modal('hide');
          (options.location || window.location).reload();
      }, this));

      this.$el.find('#settings').on('shown.bs.modal', function () {
        that.$el.keyup(function (e) {
          if (e.which == 13) {
            that.$el.find('#settings-save').click();
          }
        });
      });
      this.$el.find('#settings').on('hidden.bs.modal', function () {
        that.$el.off('keyup');
      });
  },

  load: function () {
      this.$el.find('#settings-signaling-server-url').val(this.settings.get('signaling_server').url);
  },

  save: function () {
    this.settings.set('signaling_server', {
        url: this.$el.find('#settings-signaling-server-url').val()
    });
    this.settings.save();
  },

  toggle: function () {
      this.settings.fetch();
      this.load();
      this.$el.find('#settings').modal('toggle');
  }

});

var Router = Backbone.Router.extend({

  routes: {
    'archive/:video_id' : 'archivePlay',
    'archive': 'archive',
    '': 'root'
  },

  root: function () {
    if(signaling || signalingKMS) {
      window.location.reload();
      return;
    }

    var video_id = createUUID();
    var recorder_id = createUUID();
    var peer_id = "ubuntu";

    callPeer(video_id, peer_id);
    callPeerKMS(recorder_id, peer_id);
  },

  archive: function() {

  },

  archivePlay: function(video_id) {

  }

});

RemotePlayer = Backbone.View.extend({

  initialize: function() {
    this.listenTo(this.model, 'change:stream', this.render);
  },

  render: function() {
    console.info("should be attaching media");
    var stream = this.model.get('stream');
    attachMediaStream(this.el, stream);
  },

  toggleFullscreen: function () {
    screenfull.toggle(this.el);
  }

});

Drone = Backbone.Model.extend({
  initialize: function() {
    console.info("Starting drone...");
    var ros = this.get('ros');

    this.topics = {
      nav: new ROSLIB.Topic({
        ros : ros,
        name : '/ardrone/navdata',
        messageType: 'ardrone_autonomy/Navdata'
      }),
      mission: new ROSLIB.Topic({
        ros : ros,
        name : '/ardrone_video/mission',
        messageType: 'std_msgs/String'
      }),
      takeoff: new ROSLIB.Topic({
        ros : ros,
        name : '/ardrone/takeoff',
        messageType : 'std_msgs/Empty'
      }),
      land: new ROSLIB.Topic({
        ros : ros,
        name : '/ardrone/land',
        messageType : 'std_msgs/Empty'
      }),
      reset: new ROSLIB.Topic({
        ros : ros,
        name : '/ardrone/reset',
        messageType : 'std_msgs/Empty'
      }),
      tum: new ROSLIB.Topic({
        ros: ros,
        name: '/tum_ardrone/com',
        messageType: 'std_msgs/String'
      }),
    };

    this.topics.nav.subscribe(this._onNavData.bind(this));
    this.topics.tum.subscribe(this._onTumData.bind(this));
    console.info("Started drone.");
  },

  _onNavData: function(data) {
    console.info("nav data:", data);
    this.set('nav_data', data);
  },
  _onTumData: function(data) {
    console.info("tum data: " + data);
    if(!data || !data.startsWith) return;
    if(data.startsWith('u c ')) {
      this.set('tum_path', data.substring('u c '.length));
    } else if(data.startsWith('u s ')) {
      this.set('tum_ptam', data.substring('u s '.length));
    }
  },

  takeoff: function() {
    this.topics.takeoff.publish();
  },
  land: function() {
    this.topics.land.publish();
  },
  reset: function() {
    this.topics.reset.publish();
  },
  sendMission: function(method, mission) {
    var data = method;
    for(var i = 0; i < mission.length; i++) {
      data += "\n" + mission[i].x + " " + mission[i].y;
    }
    var msg = new ROSLIB.Message({ data: data });
    this.topics.mission.publish(msg);
  },
  sendMissionRaw: function(method, mission) {
    var msg = new ROSLIB.Message({ data: method + "\n" + mission });
    this.topics.mission.publish(msg);
  },
  clearMission: function() {
    var msg = new ROSLIB.Message({ data: "c clearCommands" });
    this.topics.tum.publish(msg);
  }
});

PointMission = Backbone.Model.extend({
  defaults: {
    minimum_length: 35,
    window_radius: 5,
    neighborhood_radius: 5,
    smoothing_factor: 1.5,
    scale: 75
  },

  initialize: function() {
    this.points = [];
  },

  add: function(x, y) {
    this.points.push({ x: x, y: y });
  },
  clear: function() {
    this.points = [];
  },
  generateMission: function() {
    var points = this.points;

    var mission = [];
    var smoothing = this.get('smoothing_factor');
    var scale = this.get('scale');
    var wr = this.get('window_radius');
    var nr = this.get('neighborhood_radius');
    var min_len_sq = this.get('minimum_length');
    min_len_sq *= min_len_sq;

    var computeDistance = function(p1, p2) {
      return (p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y);
    };

    var distance=0;
    var lastPoint=points[0];
    var zero = {x:0,y:0};

    // find curvature of each point in radius of `wr`
    var curvatures = [];
    for (var i = wr; i<points.length-1-wr;i++)
    {
      var backDistance = computeDistance(points[i],points[i-wr]);
      var frontDistance = computeDistance(points[i],points[i+wr]);

      var backVector = {x:(points[i-wr].x-points[i].x)/backDistance, y:(points[i-wr].y-points[i].y)/backDistance};
      var frontVector = {x:(points[i+wr].x-points[i].x)/frontDistance, y:(points[i+wr].y-points[i].y)/frontDistance};
      var distance = computeDistance(points[i+wr],points[i-wr]);

      var cosineTheta = (backVector.x*frontVector.x+backVector.y*frontVector.y) / (backDistance*frontDistance);
      var theta = Math.acos(cosineTheta);
      var curvature = theta / distance;

      curvatures.push(curvature);
    }

    // find critical points
    var criticalPoints = [];
    for (var i = nr; i < curvatures.length-1-nr; i++)
    {
      var sum = 0;
      for (var j = -nr; j < nr; j++)
      {
        if (j != 0)
        {
          sum += curvatures[i+j];
        }
      }
      var average = sum / (nr * 2);
      if (curvatures[i] > smoothing*average)
      {
        criticalPoints.push(points[i-nr]);
      }
    }
    criticalPoints.push(points[points.length-1]);

    // create vector segments
    var startVector = points[0];
    for (var i = 0; i < criticalPoints.length; i++)
    {
      if(computeDistance(startVector, criticalPoints[i]) > min_len_sq) {
        var curVector = {
          x: (criticalPoints[i].x - startVector.x) / scale,
          y: (criticalPoints[i].y - startVector.y) / scale
        };
        console.info("Vector: (" + curVector.x + ", " + curVector.y + ")\n");
        mission.push(curVector);

        startVector = criticalPoints[i];
      }
    }

    return mission;
  }
});
PathCanvas = Backbone.View.extend({

  defaults: {
    radius: 5,
  },

  initialize: function(options) {
    this.options = _.defaults(options, this.defaults);
    console.info(this.el.constructor.name);

    this.el.width = this.el.scrollWidth;
    this.el.height = this.el.scrollHeight;

    this.center = { x: this.el.width * 0.5, y: this.el.height * 0.5 };

    this.context = this.el.getContext('2d');
    this.context.lineWidth = this.options.radius * 2;

    this.dragging = false;

    this.$el.mousedown(this._mousedown.bind(this));
    this.$el.mouseup(this._mouseup.bind(this));
    this.$el.mousemove(this._mousemove.bind(this));
  },

  _mousedown: function(e) {
    this.dragging = true;
    this._addPoint(e);
  },
  _mouseup: function(e) {
    this.dragging = false;
    this.context.beginPath();
  },
  _mousemove: function(e) {
    this._addPoint(e);
  },
  _addPoint: function(e) {
    var ctx = this.context;
    if(this.dragging) {

      //Draws line from old point to new point = new point = new mouse position
      ctx.lineTo(e.offsetX, e.offsetY);
      ctx.stroke();

      //Draws circular points - cur mouse pos, radius size, circumference of point in radians
      ctx.beginPath();
      ctx.arc(e.offsetX, e.offsetY, this.options.radius, 0, Math.PI * 2);
      ctx.fill();

      //Creates path after dot = old point = current mouse position
      ctx.beginPath();
      ctx.moveTo(e.offsetX, e.offsetY);

      this.model.add(e.offsetX - this.center.x, this.center.y - e.offsetY);
    }
  },

  clear: function() {
    this.context.clearRect(0, 0, this.el.width, this.el.height);

    $('#inputDisplay').val('');

    this.model.clear();
  },
});
DroneData = Backbone.View.extend({
  initialize: function() {
    this.listenTo(this.model, 'nav_data', this._onNavData);
    this.listenTo(this.model, 'tum_path', this._onTumPath);
    this.listenTo(this.model, 'tum_ptam', this._onTumPtam);
  },

  _onNavData(data) {
    $('#battery').html("Battery percentage: " + data.batteryPercent + "%"); 
    $('#alt').html("Estimated Altitude: " + data.altd/100 + "m"); 
    $('#vel').html("Linear Velocities: x: " + data.vx/100 + "m/s  y: " + data.vy/100 + "m/s  z : " + data.vz/100  + "m/s");
    $('#time').html("Time since started: " + data.tm/1000000 + "s"); 
  },
  _onTumPath(path) {
    $('#path').html("TUM Path: " + path);
  },
  _onTumPtam(ptam) {
    $('#ptam').html("TUM PTAM: " + ptam);
  }
});


// state

var signaling = null,
    call = null,
    signalingKMS = null,
    callKMS = null,
    remote_player = null,
    ros = null,
    drone = null,
    droneData = null,
    mission = null,
    missionCanvas = null,
    missionMethod = "tum",
    recording = false;

function initialize(dc) {
  ros = new ROSLIB.Ros();

  ros.transportLibrary = call.pc;
  ros.transportOptions = {
    reliable: false,
    ordered: false,
    protocol: 'application/vnd.rosbridge.v1+json; chunksize=512'
  };

  ros.on('connection', function() {
    console.log('Connected to server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to server closed.');
  });

  ros.connect(dc);
  call.adaptROSClient(ros);

  mission = new PointMission();
  missionCanvas = new PathCanvas({
    el: document.getElementById('imageView'),
    model: mission
  });
  drone = new Drone({
    ros: ros
  });
  droneData = new DroneData({
    mode: drone
  });
}

function callPeer(my_id, peer_id) {
  signaling = new ros_webrtc.Signaling({
    server: settings.get('signaling_server').url,
    id: my_id
  });

  signaling.once('connected', function () {
    call = new ros_webrtc.Call({
      // id: UUIDjs.create().toString().replace(/-/g, ''),
      id: UUIDjs.create().toString().replace(/-/g, ''),
      peer_id: peer_id,
      signaling: signaling
    });
    remote_player = new RemotePlayer({
      el: document.getElementById('imageTag'),
      model: call
    });
    initialize('rosbridge');
    call.dial();
  });
}
function callPeerKMS(my_id, peer_id) {
  signalingKMS = new ros_webrtc.Signaling({
    server: settings.get('signaling_server').url,
    id: my_id //UUIDjs.create().toString().replace(/-/g, '')
  });

  signalingKMS.once('connected', function () {
    callKMS = new ros_webrtc.CallKMS({
      id: UUIDjs.create().toString().replace(/-/g, ''),
      peer_id: peer_id,
      signaling: signalingKMS
    });
    callKMS.dial();
  });
}

// global

function createUUID() {
  return UUIDjs.create().toString().replace(/-/g, '');
}

var settings = new Settings();
settings.fetch();

var router = new Router();

/*var settings_dialog = new SettingsDialog({
    el: $('body').get(0),
    settings: settings
});*/

$(document).ready(function() {
  $('#recBtn').click(function () {
    console.info("hmm");
    if(callKMS) {
      if(!recording) {
        $(this).css("background", "red");
        $(this).css("color", "#black");
        callKMS.startRecording();
        recording = true;
      } else {
        $(this).css("background", "black");
        $(this).css("color", "#FFC904");
        callKMS.stopRecording();
        recording = false;
      }
    }
  });
  $('#swapBtn').click(function() {
    $('#swapBtn').html($('#imageView').is(":visible") ? "Manual" : "Canvas");
    $('#imageView').toggle();
    $('#inputDisplay').toggle();
  });
  $('#resetBtn').click(function() {
    if(drone) {
      drone.reset();
    }
  });
  $('#toggleBtn').click(function() {
    console.info("toggle:", missionMethod);
    if(missionMethod === "tum") {
      missionMethod = "noisy";
      $('#toggleBtn').html("Mode: Noisy");
    } else {
      missionMethod = "tum";
      $('#toggleBtn').html("Mode: TUM");
    }
  });
  $('#exeBtn').click(function() {
    if(drone && mission) {
      if($('#imageView').is(":visible")) {
        drone.sendMission(missionMethod, mission.generateMission());
      } else {
        drone.sendMissionRaw(missionMethod, $('#inputDisplay').val());
      }
    }
  });
  $('#landBtn').click(function() {
    if(drone) {
      drone.land();
    }
  });
  $('#takeoffBtn').click(function() {
    if(drone) {
      drone.takeoff();
    }
  });
  $('#clrBtn').click(function() {
    if(missionCanvas) {
      missionCanvas.clear();
    }
    if(drone) {
      drone.clearMission();
    }
  });

  Backbone.history.start({ root: window.location.pathname });
});
$(window).on('beforeunload', function() {
  if (call) {
    call.hangup();
  }
});


document.onkeypress = function(e) {
  if(!drone) return;

  var key = e.keyCode ? e.keyCode : e.which;
  if(key == 37) {
    drone.sendMissionRaw("manual", "west");
  } else if(key == 38) {
    drone.sendMissionRaw("manual", "north");
  } else if(key == 39) {
    drone.sendMissionRaw("manual", "east");
  } else if(key == 40) {
    drone.sendMissionRaw("manual", "south");
  }
};

document.onkeyup = function(e) {
  if(!drone) return;

  var key = e.keyCode ? e.keyCode : e.which;
  if(key == 37 || key == 38 || key == 39 || key == 40) {
    drone.sendMissionRaw("manual", "stop");
  }
}
