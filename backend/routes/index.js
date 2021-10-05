var express = require('express');
var router = express.Router();
var path = require('path');
var fs = require('fs');


/* GET home page. */
router.get('/map1', function(req, res) {
  res.send('./public/maps/map1.txt')
  // fs.readFile("./public/maps/map1.txt", "utf8", function(err, html) {
  //   const obj = JSON.stringify(html)
  // })
  // res.send(JSON.stringify(response))
});

module.exports = router;
