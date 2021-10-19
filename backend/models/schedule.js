const Sequelize = require("sequelize");

module.exports = (sequelize, DataTypes) =>
sequelize.define('schedule', {
    scheduleid: { type: DataTypes.INTEGER, autoIncrement: true, primaryKey: true },
    schedule_time: { type: DataTypes.STRING },
    schedule_title: { type: DataTypes.STRING },
    schedule_desc: { type: DataTypes.STRING },
    schedule_status: { type: DataTypes.STRING }
},{
    timestamps: false
});
