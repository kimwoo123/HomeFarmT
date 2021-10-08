-- MySQL dump 10.13  Distrib 8.0.25, for Win64 (x86_64)
--
-- Host: 52.79.134.74    Database: iot
-- ------------------------------------------------------
-- Server version	8.0.26

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `schedules`
--

DROP TABLE IF EXISTS `schedules`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `schedules` (
  `scheduleid` int NOT NULL AUTO_INCREMENT,
  `schedule_time` varchar(255) DEFAULT NULL,
  `userid` int DEFAULT NULL,
  `schedule_title` varchar(20) DEFAULT NULL,
  `schedule_desc` varchar(20) DEFAULT NULL,
  `schedule_status` varchar(20) DEFAULT NULL,
  PRIMARY KEY (`scheduleid`),
  KEY `userid` (`userid`),
  CONSTRAINT `schedules_ibfk_1` FOREIGN KEY (`userid`) REFERENCES `users` (`userid`) ON DELETE SET NULL ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=72 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `schedules`
--

LOCK TABLES `schedules` WRITE;
/*!40000 ALTER TABLE `schedules` DISABLE KEYS */;
INSERT INTO `schedules` VALUES (25,'2021-10-05T13:45',2,'감시하기',NULL,NULL),(26,'2021-10-06T02:51',2,'감시하기',NULL,NULL),(27,'2021-10-21T10:54',2,'감시하기','2번 경로','ON'),(28,'2021-10-05T03:53',2,'감시하기','2번 경로','ON'),(29,'2021-10-05T18:37',2,'감시하기','2번 경로','ON'),(30,'2021-10-07T20:32',2,'물 주기','2번 경로','OFF'),(31,'2021-10-22T00:32',2,'감시하기','2번 경로','ON'),(32,'2021-10-07T00:32',2,'감시하기','2번 경로','ON'),(33,'2021-10-22T00:32',2,'감시하기','2번 경로','ON'),(34,'2021-10-07T00:32',2,'감시하기','2번 경로','ON'),(35,'2021-10-07T00:32',2,'감시하기','2번 경로','ON'),(36,'2021-10-22T00:32',2,'감시하기','2번 경로','ON'),(37,'2021-10-22T00:32',2,'감시하기','2번 경로','ON'),(38,'2021-10-22T00:32',2,'감시하기','2번 경로','ON'),(39,'2021-10-08T13:25',17,'감시하기','','OFF'),(40,'2021-10-08T15:27',17,'제어하기','','OFF'),(41,'2021-10-12T12:28',17,'감시하기','2번 경로','ON'),(42,'2021-10-15T12:53',17,'감시하기','2번 경로','ON'),(43,'2021-10-13T11:54',17,'제어하기','1번 경로','ON'),(44,'2021-10-21T11:54',17,'제어하기','1번 경로','ON'),(45,'2021-10-21T11:54',17,'제어하기','1번 경로','ON'),(46,'2021-10-12T14:56',17,'감시하기','1번 경로','ON'),(47,'2021-10-08T17:31',17,'감시하기','1번 경로','ON'),(48,'2021-10-09T17:59',17,'감시하기','1번 경로','ON'),(49,'2021-10-09T17:59',17,'감시하기','1번 경로','OFF'),(50,'2021-10-22T02:04',17,'감시하기','1번 경로','ON'),(51,'2021-10-23T00:06',17,'감시하기','1번 경로','ON'),(52,'2021-10-11T16:12',17,'감시하기','1번 경로','ON'),(53,'2021-10-10T20:13',17,'감시하기','1번 경로','ON'),(54,'2021-10-08T21:14',17,'감시하기','1번 경로','ON'),(55,'2021-10-29T16:18',17,'감시하기','1번 경로','ON'),(56,'2021-10-13T00:20',17,'감시하기','1번 경로','ON'),(57,'2021-10-19T02:20',17,'감시하기','1번 경로','ON'),(58,'2021-10-14T15:24',17,'감시하기','1번 경로','ON'),(59,'2021-10-20T12:21',17,'감시하기','1번 경로','ON'),(60,'2021-10-20T00:54',17,'감시하기','1번 경로','ON'),(61,'2021-10-21T12:56',17,'감시하기','1번 경로','ON'),(62,'2021-10-21T12:56',17,'감시하기','1번 경로','ON'),(63,'2021-10-13T00:57',17,'감시하기','1번 경로','ON'),(64,'2021-10-22T12:59',17,'감시하기','1번 경로','ON'),(65,'2021-10-22T13:00',17,'감시하기','1번 경로','ON'),(66,'2021-10-21T13:28',17,'감시하기','2번 경로','ON'),(67,'2021-10-20T06:35',17,'감시하기','1번 경로','ON'),(68,'2021-10-08T17:38',17,'감시하기','1번 경로','ON'),(69,'2021-10-08T15:37',17,'감시하기','1번 경로','ON'),(70,'2021-10-08T02:42',17,'감시하기','1번 경로','ON'),(71,'2021-10-08T06:50',17,'감시하기','2번 경로','ON');
/*!40000 ALTER TABLE `schedules` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2021-10-08  5:48:04
